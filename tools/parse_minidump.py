    import struct
import sys
from pathlib import Path

STREAM_NAMES = {
    0: "Unused",
    3: "ThreadList",
    4: "ModuleList",
    5: "MemoryList",
    6: "Exception",
    7: "SystemInfo",
    9: "Memory64List",
    12: "HandleData",
    14: "UnloadedModule",
    15: "MiscInfo",
    16: "MemoryInfoList",
    17: "ThreadInfoList",
}

EXCEPTION_NAMES = {
    0x80000003: "STATUS_BREAKPOINT",
    0xC0000005: "STATUS_ACCESS_VIOLATION",
    0xC000001D: "STATUS_ILLEGAL_INSTRUCTION",
    0xC0000094: "STATUS_INTEGER_DIVIDE_BY_ZERO",
    0xC00000FD: "STATUS_STACK_OVERFLOW",
    0xE06D7363: "MSVC_CPP_EXCEPTION",
}


def read_cstring(f, rva: int) -> str:
    pos = f.tell()
    f.seek(rva)
    data = bytearray()
    while True:
        b = f.read(1)
        if not b or b == b"\x00":
            break
        data.extend(b)
    f.seek(pos)
    return data.decode("utf-8", "replace")


def parse(path: Path) -> None:
    with path.open("rb") as f:
        sig, ver, num_streams, dir_rva, checksum, ts, flags = struct.unpack(
            "<IIIIIQ", f.read(32)
        )
        print(f"File: {path}")
        print(f"Size: {path.stat().st_size:,} bytes")
        print(f"Signature: {sig:08X}")
        print(f"Streams: {num_streams}")
        print(f"Timestamp: {ts}")

        f.seek(dir_rva)
        streams = []
        for _ in range(num_streams):
            stype, size, rva = struct.unpack("<III", f.read(12))
            streams.append((stype, size, rva))

        for stype, size, rva in streams:
            name = STREAM_NAMES.get(stype, f"Type{stype}")
            print(f"  {name}: size={size:,}, rva=0x{rva:X}")

        exc_streams = [s for s in streams if s[0] == 6]
        if not exc_streams:
            print("No exception stream")
            return

        _, _, rva = exc_streams[0]
        f.seek(rva)
        thread_id = struct.unpack("<I", f.read(4))[0]
        f.read(4)  # alignment
        exc_code, exc_flags, exc_record, exc_addr, num_params, _align = struct.unpack(
            "<IIQII I", f.read(32)
        )
        info = struct.unpack("<15Q", f.read(15 * 8))
        params = info[:num_params]
        ctx_size, ctx_rva = struct.unpack("<II", f.read(8))

        print("--- EXCEPTION ---")
        print(f"ThreadId: {thread_id}")
        print(
            f"Code: 0x{exc_code:08X} "
            f"({EXCEPTION_NAMES.get(exc_code, 'unknown')})"
        )
        print(f"Address: 0x{exc_addr:016X}")
        print(f"Params ({num_params}): {params}")
        if exc_code == 0xC0000005 and num_params >= 2:
            op = {0: "read", 1: "write", 8: "DEP"}.get(params[0], str(params[0]))
            print(f"Access: {op} at 0x{params[1]:016X}")

        mod_streams = [s for s in streams if s[0] == 4]
        if mod_streams:
            _, _, mod_rva = mod_streams[0]
            f.seek(mod_rva)
            num_modules = struct.unpack("<I", f.read(4))[0]
            modules = []
            for _ in range(num_modules):
                base, size_m, checksum, ts_m = struct.unpack("<QIII", f.read(20))
                name_rva = struct.unpack("<I", f.read(4))[0]
                f.read(68)
                name = read_cstring(f, name_rva)
                modules.append((name, base, size_m))

            print(f"--- FAULT MODULE ---")
            for name, base, size_m in modules:
                if base <= exc_addr < base + size_m:
                    print(f"  {name}")
                    print(f"  base=0x{base:X} offset=0x{exc_addr - base:X}")
                    break
            else:
                print("  (not in any loaded module)")

            print("--- RELEVANT MODULES ---")
            keys = (
                "eu07",
                "maszyna",
                "opengl",
                "nvoglv",
                "ati",
                "igd",
                "vcruntime",
                "msvcp",
                "ntdll",
                "kernel32",
            )
            for name, base, size_m in modules:
                low = name.lower()
                if any(k in low for k in keys):
                    print(f"  {name} @ 0x{base:X}")

        thread_streams = [s for s in streams if s[0] == 3]
        if thread_streams and ctx_size:
            _, _, thread_rva = thread_streams[0]
            f.seek(thread_rva)
            num_threads = struct.unpack("<I", f.read(4))[0]
            for _ in range(num_threads):
                thread_id_t = struct.unpack("<I", f.read(4))[0]
                f.read(4)
                suspend_count = struct.unpack("<I", f.read(4))[0]
                priority_class = struct.unpack("<I", f.read(4))[0]
                priority = struct.unpack("<I", f.read(4))[0]
                teb = struct.unpack("<Q", f.read(8))[0]
                tctx_size, tctx_rva = struct.unpack("<II", f.read(8))
                f.read(24)
                if thread_id_t == thread_id and tctx_size >= 1232:
                    f.seek(tctx_rva)
                    ctx = f.read(tctx_size)
                    # CONTEXT_AMD64 offsets
                    rip = struct.unpack("<Q", ctx[248:256])[0]
                    rsp = struct.unpack("<Q", ctx[152:160])[0]
                    rbp = struct.unpack("<Q", ctx[128:136])[0]
                    rax = struct.unpack("<Q", ctx[120:128])[0]
                    rcx = struct.unpack("<Q", ctx[136:144])[0]
                    rdx = struct.unpack("<Q", ctx[144:152])[0]
                    print("--- FAULT THREAD CONTEXT ---")
                    print(f"RIP=0x{rip:016X} RSP=0x{rsp:016X} RBP=0x{rbp:016X}")
                    print(f"RAX=0x{rax:016X} RCX=0x{rcx:016X} RDX=0x{rdx:016X}")
                    for name, base, size_m in modules:
                        if base <= rip < base + size_m:
                            print(
                                f"RIP in {name} +0x{rip - base:X}"
                            )
                            break
                    break


if __name__ == "__main__":
    dump = Path(
        sys.argv[1]
        if len(sys.argv) > 1
        else r"C:\Program Files (x86)\Steam\steamapps\common\MaSzyna\crash_2026-06-11_02-31-05.dmp"
    )
    parse(dump)
