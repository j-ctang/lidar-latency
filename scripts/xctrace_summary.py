import sys, xml.etree.ElementTree as ET, collections
root=ET.parse(sys.argv[1]).getroot()
frames={}; bts={}
self_c=collections.Counter(); incl=collections.Counter(); total=0
for row in root.iter('row'):
    bt=row.find('tagged-backtrace')
    if bt is None: continue
    if 'ref' in bt.attrib: names=bts[bt.attrib['ref']]
    else:
        names=[]
        for fr in bt.findall('frame'):
            if 'ref' in fr.attrib: names.append(frames[fr.attrib['ref']])
            else: frames[fr.attrib['id']]=fr.attrib.get('name','?'); names.append(fr.attrib.get('name','?'))
        bts[bt.attrib['id']]=names
    total+=1; self_c[names[0]]+=1
    for n in set(names): incl[n]+=1
print(f"samples={total} (1 ms each)")
print("## inclusive (lidar::)")
for n,c in incl.most_common(40):
    if 'lidar' in n or n=='main': print(f"{100*c/total:6.1f}%  {c:6d}  {n[:110]}")
print("## self top 10")
for n,c in self_c.most_common(10): print(f"{100*c/total:6.1f}%  {c:6d}  {n[:110]}")
