from xml.etree.ElementTree import parse
import os
from PIL import Image
import sys

xmls = os.listdir("./n04120489/")
os.chdir("./n04120489")

pics = os.listdir("../../Tx2-RCCAR/shoes_data/shoe-pics/")

for xml in xmls:
    sys.stdout.flush()
    tree = parse(xml)
    note = tree.getroot()
    print(xml)
    imgpath = r"../../Tx2-RCCAR/shoes_data/shoe-pics/" + xml[:-4] + ".jpg"
    img = None
    try:
        img = Image.open(imgpath)
    exce`pt:
        continue
    txt = open("../" + xml[:-4] + ".txt" , "w")
    dw = 1./img.size[0]
    dh = 1./img.size[1]
    for bnd in  note.iter("bndbox"):
       Xcenter = ((float(bnd.findtext('xmax')) + float(bnd.findtext('xmin'))) / 2.0) * dw
       Ycenter = ((float(bnd.findtext('ymax')) + float(bnd.findtext('ymin'))) / 2.0) * dh
       width = (float(bnd.findtext('xmax')) - float(bnd.findtext('xmin'))) * dw
       height = (float(bnd.findtext('ymax')) - float(bnd.findtext('ymin'))) * dh
       datas = [Xcenter, Ycenter, width, height]
       txt.write('0 ' + ' '.join([str(data) for data in datas]) + '\n')

    txt.close()
