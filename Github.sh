#！/bin/bash
echo "Github Push Demo"
git add .
git commit -m "$(date) update"
export http_proxy=http://127.0.0.1:1087;export https_proxy=http://127.0.0.1:1087;git push -f origin master