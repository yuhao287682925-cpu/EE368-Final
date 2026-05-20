import zipfile
import xml.etree.ElementTree as ET
import sys

def get_text_from_pptx(filename, output_file):
    with zipfile.ZipFile(filename, 'r') as z:
        slides = [f for f in z.namelist() if f.startswith('ppt/slides/slide') and f.endswith('.xml')]
        slides = [f for f in slides if f.split('/')[-1].replace('slide', '').replace('.xml', '').isdigit()]
        slides.sort(key=lambda x: int(x.split('/')[-1].replace('slide', '').replace('.xml', '')))
        
        with open(output_file, 'w', encoding='utf-8') as f_out:
            for slide in slides:
                f_out.write(f"--- {slide} ---\n")
                content = z.read(slide)
                root = ET.fromstring(content)
                for elem in root.iter():
                    if elem.tag.endswith('}t'):
                        if elem.text:
                            f_out.write(elem.text + '\n')

get_text_from_pptx("ee386第10组开题.pptx", "extracted_text.txt")
