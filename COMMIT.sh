#!/bin/sh

git config user.name "Iranaphor"
git config user.email "Primordia@live.com"

git config --global user.name "Iranaphor"
git config --global user.email "Primordia@live.com"

git add .

read -p 'Commit Message: ' msg
read -p 'Commit Description: ' desc

echo $msg
echo $desc

git commit -m "$msg" -m "$desc"
git push origin 


git config user.name unset
git config user.email unset
git config --global user.name unset
git config --global user.email unset
