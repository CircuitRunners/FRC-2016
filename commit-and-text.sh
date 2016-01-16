git add . --all
git commit
git push
curl -X POST http://textbelt.com/text -d number=6783435072 -d "message=We pushed things to repo!"
