git reset --hard v4.0
git checkout --orphan test
git commit -m "Truncated History"
git rebase --onto test v4.0..master


1. cd /path/to/git/localrepo
2. svn mkdir --parents protocol:///path/to/repo/PROJECT/trunk -m "Importing git repo"
3. git svn init protocol:///path/to/repo/PROJECT -s
4. git svn fetch
5. git rebase trunk
5.1.  git status
5.2.  git add (conflicted-files)
5.3.  git rebase --continue
5.4.  (repeat 5.1.)
6. git svn dcommit
