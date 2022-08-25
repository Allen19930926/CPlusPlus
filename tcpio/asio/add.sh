git clean -df
git checkout -- test/__pycache__/
git checkout -- test/pytest.log
git checkout -- app/infrastructure/common/version.h
git st
git add .
git commit --amend
git st