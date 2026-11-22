/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "utilities/crashsignals.h"

#if defined(__linux__)

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <system_error>

#include <execinfo.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

namespace {

// the handler may only call what is safe in a signal: no allocation, no locks, no streams. so what it needs is
// kept in fixed buffers, filled in beforehand, and written with write()

constexpr std::array crash_signal_list{SIGSEGV, SIGBUS, SIGFPE, SIGILL, SIGABRT};
// a handler that hangs (a crash inside the allocator or the loader, which backtrace() may need) ends this long after
constexpr unsigned int crash_report_timeout_seconds{5};

// a text of fixed size, filled outside the handler and read in it
template <std::size_t Size> struct fixed_text {
	std::array<char, Size> text{};
	std::atomic<bool> ready{false};

	void set(std::string_view const Text) {
		ready = false;
		auto const length{std::min(Text.size(), text.size() - 1)};
		std::copy_n(Text.data(), length, text.data());
		text[length] = '\0';
		ready = true;
	}
	char const *get() const { return ready ? text.data() : nullptr; }
};

fixed_text<1024> &log_path() {
	static fixed_text<1024> path;
	return path;
}

fixed_text<256> &version_text() {
	static fixed_text<256> version;
	return version;
}

// the stack the handler runs on, so an overflow of the main thread's own stack is reported too
std::array<char, 64 * 1024> &alternate_stack() {
	static std::array<char, 64 * 1024> stack{};
	return stack;
}

void write_all(int const File, std::string_view Text) {
	while (false == Text.empty()) {
		auto const written{::write(File, Text.data(), Text.size())};
		if (written <= 0)
			return;
		Text.remove_prefix(static_cast<std::size_t>(written));
	}
}

void write_hex(int const File, std::uintptr_t const Value) {
	constexpr std::size_t digits{sizeof(Value) * 2};
	constexpr std::string_view hexdigits{"0123456789abcdef"};
	std::array<char, digits + 2> text{'0', 'x'};
	for (std::size_t digit = 0; digit < digits; ++digit)
		text[2 + digit] = hexdigits[(Value >> (4 * (digits - 1 - digit))) & 0xF];
	write_all(File, {text.data(), text.size()});
}

void write_decimal(int const File, int const Value) {
	std::array<char, 16> text{};
	auto position{text.size()};
	auto rest{static_cast<unsigned int>(Value < 0 ? -Value : Value)};
	do {
		text[--position] = static_cast<char>('0' + rest % 10);
		rest /= 10;
	} while (rest != 0 && position > 1);
	if (Value < 0)
		text[--position] = '-';
	write_all(File, {text.data() + position, text.size() - position});
}

std::string_view signal_name(int const Signal) {
	switch (Signal) {
		case SIGSEGV:
			return "SIGSEGV";
		case SIGBUS:
			return "SIGBUS";
		case SIGFPE:
			return "SIGFPE";
		case SIGILL:
			return "SIGILL";
		case SIGABRT:
			return "SIGABRT";
		default:
			return "?";
	}
}

// the signal being reported, for a report that hangs
std::atomic<int> &crash_signal() {
	static std::atomic<int> signal{0};
	return signal;
}

// a report that hangs is given up: the process ends by the signal of the crash, as it would have without the handler
void handle_report_timeout(int) {
	auto const crashsignal{crash_signal().load()};
	raise(crashsignal);
	// it is blocked while its own handler runs, which this one interrupted
	sigset_t unblocked;
	sigemptyset(&unblocked);
	sigaddset(&unblocked, crashsignal);
	sigprocmask(SIG_UNBLOCK, &unblocked, nullptr);
}

struct crash_stack {
	std::array<void *, 64> frames{};
	int count{0};
};

void write_report(int const File, int const Signal, siginfo_t const *Info, crash_stack const &Stack) {
	write_all(File, "\nCrash: signal ");
	write_decimal(File, Signal);
	write_all(File, " (");
	write_all(File, signal_name(Signal));
	write_all(File, ")");
	// the address of the fault; an abort has none
	if (Signal != SIGABRT && Info != nullptr) {
		write_all(File, " at address ");
		write_hex(File, reinterpret_cast<std::uintptr_t>(Info->si_addr));
	}
	write_all(File, "\n");
	if (auto const *version{version_text().get()}) {
		write_all(File, "Version: ");
		write_all(File, version);
		write_all(File, "\n");
	}
	write_all(File, "Stack (addr2line -f -C -e <the eu07 file of this version> <the offset after the +>):\n");
	backtrace_symbols_fd(Stack.frames.data(), Stack.count, File);
}

void handle_crash_signal(int const Signal, siginfo_t *Info, void *) {
	crash_signal() = Signal;
	struct sigaction timeout{};
	timeout.sa_handler = handle_report_timeout;
	sigemptyset(&timeout.sa_mask);
	sigaction(SIGALRM, &timeout, nullptr);
	alarm(crash_report_timeout_seconds);

	crash_stack stack;
	stack.count = backtrace(stack.frames.data(), static_cast<int>(stack.frames.size()));

	if (auto const *path{log_path().get()}) {
		if (auto const log{::open(path, O_WRONLY | O_APPEND | O_CLOEXEC)}; log >= 0) {
			write_report(log, Signal, Info, stack);
			::close(log);
		}
	}
	write_report(STDERR_FILENO, Signal, Info, stack);

	// the handler was taken off as it was called (SA_RESETHAND): the signal does what it always did
	raise(Signal);
}

} // namespace

namespace crash_signals {

void install(std::string_view const Version) {
	static std::atomic<bool> installed{false};
	if (installed.exchange(true))
		return;

	version_text().set(Version);

	// backtrace() loads what it needs on its first call, which allocates: done here, not in the handler
	std::array<void *, 1> warmup{};
	backtrace(warmup.data(), static_cast<int>(warmup.size()));

	stack_t alternate{};
	alternate.ss_sp = alternate_stack().data();
	alternate.ss_size = alternate_stack().size();
	sigaltstack(&alternate, nullptr);

	struct sigaction action{};
	action.sa_sigaction = handle_crash_signal;
	action.sa_flags = SA_SIGINFO | SA_ONSTACK | SA_RESETHAND;
	// another crash signal of the same thread waits until the report is written, rather than starting a second one in
	// its middle (a fault that comes while it is blocked still ends the process). a crash of another thread at the
	// same moment may still write its report alongside
	sigemptyset(&action.sa_mask);
	for (auto const crashsignal : crash_signal_list)
		sigaddset(&action.sa_mask, crashsignal);
	for (auto const crashsignal : crash_signal_list)
		sigaction(crashsignal, &action, nullptr);
}

void set_log_path(std::string_view const Path) {
	// whole, so the report finds the file wherever the simulator stands then
	std::error_code error;
	auto const path{std::filesystem::absolute(std::filesystem::path{Path}, error)};
	log_path().set(error ? std::string{Path} : path.string());
}

} // namespace crash_signals

#else

namespace crash_signals {

void install(std::string_view) {}
void set_log_path(std::string_view) {}

} // namespace crash_signals

#endif
