"""Keep repo-relative Markdown usable in GitHub and the rendered team site."""
import re
from pathlib import Path


def on_page_markdown(markdown, page, config, **kwargs):
    docs = Path(config['docs_dir']).resolve()
    root = docs.parent
    source = Path(page.file.abs_src_path).parent
    # Use the same branch as edit_uri, which can itself contain slashes.
    branch = config['edit_uri'].removeprefix('edit/').removesuffix('/docs/')
    repo = config['repo_url'].rstrip('/')

    def rewrite(match):
        target = match.group(1)
        if '://' in target or target.startswith(('#', 'mailto:')):
            return match.group(0)
        path, marker, anchor = target.partition('#')
        resolved = (source / path).resolve()
        if resolved.is_relative_to(root):
            if resolved.is_dir():
                kind = 'tree'
            elif resolved.is_file() and not resolved.is_relative_to(docs):
                kind = 'blob'
            else:
                return match.group(0)
            base = repo + '/' + kind + '/' + branch + '/'
            return '](' + base + resolved.relative_to(root).as_posix() + marker + anchor + ')'
        return match.group(0)

    return re.sub(r'\]\(([^\s)]+)\)', rewrite, markdown)
