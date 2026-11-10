// the minidump writer shared by the simulator and eu07_starter, see utilities/crashdump_channel.h

#ifdef _WIN32

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <DbgHelp.h>

#include "utilities/crashdump_channel.h"

namespace
{

struct dump_context
{
	ULONG64 main_module_base;
};

BOOL CALLBACK dump_callback(PVOID Parameter, PMINIDUMP_CALLBACK_INPUT const Input, PMINIDUMP_CALLBACK_OUTPUT Output)
{
	if ((Input == nullptr) || (Output == nullptr))
	{
		return FALSE;
	}
	switch (Input->CallbackType)
	{
	case IncludeModuleCallback:
	case IncludeThreadCallback:
	case ThreadCallback:
	case ThreadExCallback:
		return TRUE;
	case ModuleCallback:
	{
		// the data sections of the other modules, the graphics driver above all, take tens of megabytes and tell nothing about a crash of the simulator.
		// its own globals stay in the dump
		auto const *context{static_cast<dump_context const *>(Parameter)};
		if (Input->Module.BaseOfImage != context->main_module_base)
		{
			Output->ModuleWriteFlags &= ~ModuleWriteDataSeg;
		}
		return TRUE;
	}
	default:
		return FALSE;
	}
}

} // namespace

bool crashdump_write_minidump(void *Process, crashdump::channel_data &Data, bool const ClientPointers)
{
	Data.dump_path[crashdump::path_capacity - 1] = L'\0';
	auto const file{::CreateFileW(Data.dump_path, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr)};
	if (file == INVALID_HANDLE_VALUE)
	{
		Data.error = ::GetLastError();
		Data.state = crashdump::dump_failed;
		return false;
	}

	// the full dump, on request of crashdump.full in eu07.ini, has the whole memory of the process: hundreds of megabytes or more.
	// the small one the stacks of all threads with the memory their pointers lead to, the globals of the simulator and the map of the memory
	auto const type{static_cast<MINIDUMP_TYPE>(
	    (Data.full_memory != 0 ? MiniDumpWithFullMemory : (MiniDumpWithDataSegs | MiniDumpWithIndirectlyReferencedMemory)) | MiniDumpWithHandleData | MiniDumpWithFullMemoryInfo |
	    MiniDumpWithThreadInfo | MiniDumpWithUnloadedModules | MiniDumpWithProcessThreadData | MiniDumpIgnoreInaccessibleMemory)};

	MINIDUMP_EXCEPTION_INFORMATION exception{};
	exception.ThreadId = Data.thread_id;
	exception.ExceptionPointers = reinterpret_cast<PEXCEPTION_POINTERS>(static_cast<ULONG_PTR>(Data.exception_pointers));
	exception.ClientPointers = ClientPointers ? TRUE : FALSE;

	auto const commentlength{Data.comment_length < crashdump::comment_capacity ? Data.comment_length : static_cast<std::uint32_t>(crashdump::comment_capacity)};
	MINIDUMP_USER_STREAM comment{};
	comment.Type = CommentStreamA;
	comment.BufferSize = commentlength;
	comment.Buffer = Data.comment;
	MINIDUMP_USER_STREAM_INFORMATION streams{};
	streams.UserStreamCount = commentlength > 0 ? 1 : 0;
	streams.UserStreamArray = &comment;

	dump_context context{Data.main_module_base};
	MINIDUMP_CALLBACK_INFORMATION callback{};
	callback.CallbackRoutine = dump_callback;
	callback.CallbackParam = &context;

	auto const result{::MiniDumpWriteDump(Process, Data.process_id, file, type, Data.exception_pointers != 0 ? &exception : nullptr, &streams, Data.full_memory != 0 ? nullptr : &callback)};
	Data.error = result ? 0 : ::GetLastError();
	::CloseHandle(file);
	Data.state = result ? crashdump::dump_written : crashdump::dump_failed;
	return result != FALSE;
}

#endif
