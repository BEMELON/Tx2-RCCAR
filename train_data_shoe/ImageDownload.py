import urllib.request
import wget
import os
response = urllib.request.urlopen('http://www.image-net.org/api/text/imagenet.synset.geturls.getmapping?wnid=n04120489')
html = response.read()
html = html.decode('utf-8')
'\n'.join(html.splitlines())

line = 0
while True:

    new_line = html.find('\n', line)
    if(new_line == line + 1): break
    line = html[line:new_line]
    idx = line.find('h')
#   os.system('wget --tries=1 -O ' + line[0:idx-1] + '.jpg -P ./shoe-pics/ ' + line[idx:])
    os.system('curl ' + line[idx:-1] +  ' > ' + line[0:idx-1] + '.jpg')
#    wget.download(line[idx:], 'shoe-pics/' + line[0:idx-1] + '.jpg')
    line = new_line + 1



