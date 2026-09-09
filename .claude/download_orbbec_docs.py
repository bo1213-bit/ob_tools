from __future__ import annotations

import hashlib
import os
import re
import sys
import time
from collections import deque
from html.parser import HTMLParser
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import unquote, urljoin, urlsplit, urlunsplit
from urllib.request import Request, urlopen

START_URL = "https://orbbec.github.io/OrbbecSDK_ROS2/en/index.html"
OUTPUT_ROOT = Path(r"D:\Data\robotPackage\ob_tools\资料\OrbbecSDK_ROS2_docs")
ALLOWED_HOST = "orbbec.github.io"
ALLOWED_PATH_PREFIX = "/OrbbecSDK_ROS2/"
USER_AGENT = "Mozilla/5.0 (compatible; LocalDocumentationArchive/1.0)"


class LinkCollector(HTMLParser):
    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.links: list[str] = []

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        for name, value in attrs:
            if value and name.lower() in {"href", "src", "poster", "data-src"}:
                self.links.append(value)


def normalized_url(url: str) -> str | None:
    parts = urlsplit(url)
    if parts.scheme not in {"http", "https"} or parts.netloc.lower() != ALLOWED_HOST:
        return None
    path = parts.path or "/"
    if not path.startswith(ALLOWED_PATH_PREFIX):
        return None
    return urlunsplit(("https", ALLOWED_HOST, path, parts.query, ""))


def output_path(url: str, content_type: str) -> Path:
    parts = urlsplit(url)
    relative = unquote(parts.path.lstrip("/"))
    if relative.endswith("/"):
        relative += "index.html"
    if not Path(relative).suffix and "text/html" in content_type:
        relative += ".html"
    if parts.query:
        stem, suffix = os.path.splitext(relative)
        relative = f"{stem}__q_{hashlib.sha256(parts.query.encode()).hexdigest()[:12]}{suffix}"
    return OUTPUT_ROOT / relative


def fetch(url: str) -> tuple[bytes, str]:
    request = Request(url, headers={"User-Agent": USER_AGENT})
    with urlopen(request, timeout=45) as response:
        return response.read(), response.headers.get_content_type()


def main() -> int:
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    queue: deque[str] = deque([START_URL])
    queued = {START_URL}
    completed = set()
    errors: list[str] = []

    while queue:
        url = queue.popleft()
        if url in completed:
            continue
        try:
            body, content_type = fetch(url)
            destination = output_path(url, content_type)
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(body)
            completed.add(url)
            print(f"OK {url} -> {destination}", flush=True)

            if content_type == "text/html":
                parser = LinkCollector()
                parser.feed(body.decode("utf-8", errors="replace"))
                for raw_link in parser.links:
                    candidate = normalized_url(urljoin(url, raw_link))
                    if candidate and candidate not in queued and candidate not in completed:
                        queued.add(candidate)
                        queue.append(candidate)
            time.sleep(0.05)
        except (HTTPError, URLError, TimeoutError, OSError) as exc:
            errors.append(f"{url}: {exc}")
            print(f"ERROR {url}: {exc}", flush=True)

    (OUTPUT_ROOT / "download-errors.txt").write_text("\n".join(errors) + ("\n" if errors else ""), encoding="utf-8")
    (OUTPUT_ROOT / "README.txt").write_text(
        f"Local mirror of {START_URL}\nDownloaded files: {len(completed)}\nFailed requests: {len(errors)}\n"
        "Open OrbbecSDK_ROS2/en/index.html to browse the saved documentation.\n",
        encoding="utf-8",
    )
    print(f"Finished: {len(completed)} files downloaded, {len(errors)} requests failed.", flush=True)
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main())
