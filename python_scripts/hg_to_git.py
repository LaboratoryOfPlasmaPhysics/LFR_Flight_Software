from glob import glob
import subprocess
import os
import argparse
import traceback
import re

version_regex=re.compile('[a-zA-Z ]{0,4}(\d+).(\d+).(\d+).(\d+).*')

def load_hg_history(raw_history):
    history = []
    current_entry = None
    loading_description = False
    for line in raw_history:
        if line.startswith('changeset:'):
            loading_description = False
            field,value = line.split(':',1)
            if current_entry:
                history.append(current_entry)
            index,value = value.split(':')
            current_entry = {'index':index}
            current_entry[field]=value
        elif loading_description:
            current_entry['description'] += '\n' + line
        elif ':' in line:
            field,value = line.split(':',1)
            if field == 'description':
                loading_description = True
            current_entry[field]=value.strip()
    history.reverse()
    return history

def generate_gitignore(path):
    with open(f"{path}/.gitignore", 'w') as gi:
        gi.write("""
*.user
.hg*
*/.hg*
*.cbp
*.cbTemp
*.srec
*.layout
*.depend
*.user
*.user.*
*.sublime-*
.buildconfig
*/bin/Debug/*
*/bin/fsw*
*/bin/timegen
Makefile
        """)


def hg_clean(hg_path):
    cmd = ["hg", "clean", "--all"]
    subprocess.run(cmd, cwd=hg_path)

def hg_update(hg_path, changeset):
    hg_clean(hg_path)
    cmd = ["hg", "update", "--clean", "-r", changeset]
    subprocess.run(cmd, cwd=hg_path)
    hg_clean(hg_path)

def git_switch_branch(git_path, branch):
    cmd = ['git', 'checkout', '-b', branch]
    subprocess.run(cmd, cwd=git_path)

def sync_from_hg_to_git(hg_path, git_path):
    cmd = ['rsync', '-a', "--delete", "--cvs-exclude", "--exclude=.gitignore", "--exclude=.hgignore", f"{hg_path}/", git_path]
    subprocess.run(cmd)
    deleted_files = subprocess.run(["git", "ls-files", "--deleted"], cwd=git_path, stdout=subprocess.PIPE, text=True).stdout.split()
    if len(deleted_files):
        subprocess.run(['git', 'rm'] + deleted_files, cwd=git_path)
    cmd = ['git', 'add', '*']
    subprocess.run(cmd, cwd=git_path)

def git_commit(git_path, date, author_name, author_mail, message, label=None):
    cmd = ['git', 'commit', '-m', message]
    env = {
            'GIT_AUTHOR_DATE':date,
            'GIT_COMMITTER_DATE':date,
            'GIT_COMMITTER_NAME':author_name,
            'GIT_COMMITTER_EMAIL':author_mail,
            'GIT_AUTHOR_NAME':author_name,
            'GIT_AUTHOR_EMAIL':author_mail
            }
    subprocess.run(cmd, cwd=git_path, env=env)
    if label is not None:
        subprocess.run(['git', 'tag', label], cwd=git_path, env=env)


def map_user(user):
    if 'paul' in user.lower():
        return 'Paul Leroy', 'paul.leroy@univ-rennes1.fr'
    if 'leroy' in user.lower():
        return 'Paul Leroy', 'paul.leroy@univ-rennes1.fr'
    if 'alexis' in user.lower():
        return 'Alexis Jeandet', 'alexis.jeandet@member.fsf.org'
    if 'jeandet' in user.lower():
        return 'Alexis Jeandet', 'alexis.jeandet@member.fsf.org'
    raise

def extract_tag(revision):
    tag = revision.get('tag', None)
    if tag is None:
        match = version_regex.match(revision['description'].split()[0])
        if match:
            tag = '.'.join(match.groups())
    return tag

def build_git_repo(history, hg_path, git_path):
    subprocess.run(["rm","-rf", git_path])
    subprocess.run(["mkdir","-p", git_path])
    subprocess.run(["git","init", git_path])
    generate_gitignore(git_path)
    for revision in history:
        try:
            branch = revision.get('branch', 'main')
            tag = extract_tag(revision)
            message = revision['description']
            date = revision['date']
            auth_name, auth_mail = map_user(revision['user'])

            hg_update(hg_path, revision['changeset'])
            git_switch_branch(git_path,branch)
            sync_from_hg_to_git(hg_path, git_path)
            git_commit(git_path, date, auth_name, auth_mail, message, tag)
        except:
            print(revision)
            print(traceback.format_exc())

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--source-repo", help="Source repo")
    parser.add_argument("-d", "--dest-repo", help="Destination repo")
    args = parser.parse_args()

    source_repo = args.source_repo
    dest_repo = args.dest_repo

    hg_raw_history = subprocess.run(["hg", "-v", "log", source_repo], stdout=subprocess.PIPE, text=True).stdout.split('\n')
    #print(hg_raw_history)
    history = load_hg_history(hg_raw_history)
    #print(history)
    build_git_repo(history[:], source_repo, dest_repo)
