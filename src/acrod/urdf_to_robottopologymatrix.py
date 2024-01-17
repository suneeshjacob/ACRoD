import numpy
import xml.etree.ElementTree as ET

def get_joints_from_urdf(root):
    joints_info = []
    for child in root:
        if child.tag == 'joint':
            if child.attrib['type'] in ['revolute', 'prismatic', 'planar']:
                parent_link = None
                child_link = None
                for child2 in child:
                    if child2.tag == 'parent':
                        parent_link = child2.attrib['link']
                    if child2.tag == 'child':
                        child_link = child2.attrib['link']
                joints_info.append((child.attrib['name'], child.attrib['type'], parent_link, child_link,))
    
    return {i[0]:i[1:] for i in sorted(joints_info, key=lambda x:x[0])}

def get_links_from_urdf(root):
    links_info = []
    for child in root:
        if child.tag == 'link':
            links_info.append(child.attrib['name'])
    return numpy.unique(sorted(links_info))


filepath = r'/Users/apple/Downloads/T12.URDF'
tree = ET.parse(filepath)
root = tree.getroot()

all_links = get_links_from_urdf(root)
all_joints = get_joints_from_urdf(root)

parent_links = set(list(zip(*all_joints.values()))[1])
child_links = set(list(zip(*all_joints.values()))[2])

base_links = parent_links - child_links
endeffector_links = child_links - parent_links

base_link = sorted(base_links)[0]
endeffector_link = sorted(endeffector_links)[0]

n = len(all_links)
M = 9*numpy.eye(n)



links_list = [base_link] + all_links[numpy.logical_and(all_links != base_link, all_links != endeffector_link)].tolist() + [endeffector_link]
links_dict = dict(enumerate(links_list))
