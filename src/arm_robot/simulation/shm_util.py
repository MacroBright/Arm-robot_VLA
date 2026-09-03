"""POSIX 共享内存 — 进程崩溃不消失, 比 Python shared_memory 可靠。"""
import mmap, os

SHM_DIR = "/dev/shm"

def create(name: str, size: int) -> mmap.mmap:
    path = f"{SHM_DIR}/{name}"
    try: os.unlink(path)
    except FileNotFoundError: pass
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o600)
    os.ftruncate(fd, size)
    shm = mmap.mmap(fd, size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
    os.close(fd)
    return shm

def open_read(name: str, size: int) -> mmap.mmap | None:
    try:
        fd = os.open(f"{SHM_DIR}/{name}", os.O_RDONLY)
        shm = mmap.mmap(fd, size, mmap.MAP_SHARED, mmap.PROT_READ)
        os.close(fd)
        return shm
    except FileNotFoundError:
        return None

def close(shm: mmap.mmap) -> None:
    shm.close()

def unlink(name: str) -> None:
    try: os.unlink(f"{SHM_DIR}/{name}")
    except FileNotFoundError: pass
