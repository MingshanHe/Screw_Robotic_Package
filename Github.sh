#！/bin/bash
echo "Github Push Demo"
git add .
git commit -m "$(date) update"
git push -f origin master