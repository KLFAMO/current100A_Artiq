#!/usr/bin/env python3
"""
pnc.py — simple nc-like TCP client:
- for each user line: open TCP -> send -> receive -> close
- default: port 10, EOL=LF (no CR), timeout 0.01 s
- works on both Windows and Linux

Examples:
  python pnc.py 192.168.3.44
  python pnc.py 192.168.3.44 10 --timeout 3 -v --force-nl
  echo "STATUS ?" | python pnc.py 192.168.3.44 --hex
"""

import socket
import argparse
import sys
import signal

def make_parser():
    p = argparse.ArgumentParser(
        description="pnc - simple nc-like client (open-send-recv-close per line)"
    )
    p.add_argument("host", help="target IP address or hostname")
    p.add_argument("port", nargs="?", type=int, default=10,
                   help="TCP port (default: 10)")
    p.add_argument(
        "--eol", choices=["NONE", "LF", "CR", "CRLF"], default="LF",
        help="end-of-line characters to append to sent text (default: LF)"
    )
    p.add_argument(
        "--timeout", type=float, default=0.01,
        help="timeout in seconds for reading a reply (default: 0.01)"
    )
    p.add_argument("--prompt", default="> ", help="prompt for interactive mode")
    # Diagnostic and behavior flags
    p.add_argument("--shutdown", action="store_true",
                   help="call shutdown(SHUT_WR) after sending (default: disabled)")
    p.add_argument("--verbose", "-v", action="store_true",
                   help="print debug info to stderr")
    p.add_argument("--hex", action="store_true",
                   help="print the hexadecimal representation of the reply")
    p.add_argument("--force-nl", action="store_true",
                   help="force newline after response if it doesn’t end with '\\n'")
    return p

EOL_MAP = {
    "NONE": "",
    "LF": "\n",
    "CR": "\r",
    "CRLF": "\r\n",
}

def send_once(host, port, payload_bytes, timeout, do_shutdown=False, verbose=False):
    """Open connection, send payload, receive response (until timeout), then close."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        if verbose:
            print(f"[DBG] Connecting to {host}:{port}", file=sys.stderr)
        s.connect((host, port))

        if verbose:
            print(f"[DBG] Sending {len(payload_bytes)} bytes", file=sys.stderr)
        s.sendall(payload_bytes)

        if do_shutdown:
            if verbose:
                print("[DBG] shutdown(SHUT_WR)", file=sys.stderr)
            try:
                s.shutdown(socket.SHUT_WR)
            except OSError:
                pass

        chunks = []
        while True:
            try:
                data = s.recv(4096)
                if not data:
                    if verbose:
                        print("[DBG] Peer closed connection", file=sys.stderr)
                    break
                chunks.append(data)
            except socket.timeout:
                if verbose:
                    print("[DBG] Receive timeout", file=sys.stderr)
                break

        return b"".join(chunks)

    finally:
        try:
            s.close()
        except Exception:
            pass

def interactive_loop(host, port, eol_bytes, timeout, prompt, do_shutdown, verbose, show_hex, force_nl):
    # Graceful exit on Ctrl+C
    def sigint_handler(sig, frame):
        print("\nExit.")
        sys.exit(0)
    try:
        signal.signal(signal.SIGINT, sigint_handler)
    except Exception:
        # May not work in some Windows environments — ignore
        pass

    # Non-interactive mode (piped input)
    if not sys.stdin.isatty():
        for line in sys.stdin:
            line = line.rstrip("\n")
            payload = line.encode("utf-8") + eol_bytes
            try:
                resp = send_once(host, port, payload, timeout, do_shutdown, verbose)
                if resp:
                    sys.stdout.write(resp.decode("utf-8", errors="replace"))
                    if show_hex:
                        sys.stdout.write("\n[HEX] " + resp.hex() + "\n")
                    if force_nl and not resp.endswith(b"\n"):
                        sys.stdout.write("\n")
                    sys.stdout.flush()
            except Exception as e:
                print(f"[ERROR] {e}", file=sys.stderr)
        return

    # Interactive prompt mode
    while True:
        try:
            line = input(prompt)
        except EOFError:
            print()
            break
        except KeyboardInterrupt:
            print()
            break

        payload = line.encode("utf-8") + eol_bytes
        try:
            resp = send_once(host, port, payload, timeout, do_shutdown, verbose)
            if resp:
                sys.stdout.write(resp.decode("utf-8", errors="replace"))
                if show_hex:
                    sys.stdout.write("\n[HEX] " + resp.hex() + "\n")
                if force_nl and not resp.endswith(b"\n"):
                    sys.stdout.write("\n")
                sys.stdout.flush()
        except Exception as e:
            print(f"[ERROR] {e}", file=sys.stderr)

def main():
    args = make_parser().parse_args()
    eol_bytes = EOL_MAP[args.eol].encode("utf-8")
    interactive_loop(
        host=args.host,
        port=args.port,
        eol_bytes=eol_bytes,
        timeout=args.timeout,
        prompt=args.prompt,
        do_shutdown=args.shutdown,
        verbose=args.verbose,
        show_hex=args.hex,
        force_nl=args.force_nl,
    )

if __name__ == "__main__":
    main()
