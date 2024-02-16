import xml.etree.ElementTree as ET
import os


def select_tags(root, node_list, way_list, relation_list) -> None:
    for child in root:
        if child.tag == "node":
            node_list.append(child)
        elif child.tag == "way":
            way_list.append(child)
        elif child.tag == "relation":
            relation_list.append(child)
        else:
            print("Unknown tag: {}".format(child.tag))


if __name__ == "__main__":
    whole_lanelet = "/home/bzeren/projects/lanelet2-component/osmium/osmium-test/sorted.osm"
    divided_lanelet = "/home/bzeren/projects/lanelet2-component/osmium/osmium-test/585431.osm"

    whole_map_xml = ET.parse(whole_lanelet)
    whole_map_root = whole_map_xml.getroot()

    node_list = []
    way_list = []
    relation_list = []

    select_tags(whole_map_root, node_list, way_list, relation_list)

    # read file names from folder
    osm_files = [f for f in
                 os.listdir("/home/bzeren/projects/lanelet2-component/dummy-lanelet2-generator/lanelet2_maps/") if
                 f.endswith(".osm")]
    print(osm_files)

    for osm_filename in osm_files:
        divided_map_xml = ET.parse("/home/bzeren/projects/lanelet2-component/dummy-lanelet2-generator/lanelet2_maps/" + osm_filename)
        divided_map_root = divided_map_xml.getroot()

        divided_node_list = []
        divided_way_list = []
        divided_relation_list = []

        select_tags(divided_map_root, divided_node_list, divided_way_list, divided_relation_list)

        # Iterate on divided map and find missing members
        for relation in divided_relation_list:
            relation_refs = [member for member in relation.iter("member")]
            for r in relation_refs:
                if r.attrib["ref"] not in [way.attrib["id"] for way in divided_way_list] and r.attrib["ref"] not in [
                    node.attrib["id"] for node in divided_relation_list]:
                    # find the way or relation in the whole map and add it to the divided map
                    if r.attrib["type"] == "way":
                        for way in way_list:
                            if way.attrib["id"] == r.attrib["ref"]:
                                divided_map_root.append(way)
                    elif r.attrib["type"] == "relation":
                        for relation in relation_list:
                            if relation.attrib["id"] == r.attrib["ref"]:
                                divided_map_root.append(relation)
                    select_tags(divided_map_root, divided_node_list, divided_way_list, divided_relation_list)

        # Iterate on divided map's ways and find missing nodes
        for way in divided_way_list:
            nd = [nd.attrib["ref"] for nd in way.iter("nd")]
            for n in nd:
                if n not in [node.attrib["id"] for node in divided_node_list]:
                    # find the node in the whole map and add it to the divided map
                    for node in node_list:
                        if node.attrib["id"] == n:
                            divided_map_root.append(node)
                    select_tags(divided_map_root, divided_node_list, divided_way_list, divided_relation_list)

        divided_map_xml.write("/home/bzeren/projects/lanelet2-component/dummy-lanelet2-generator/lanelet2_maps/" + osm_filename)

    # # Divided map
    # divided_map_xml = ET.parse(divided_lanelet)
    # divided_map_root = divided_map_xml.getroot()
    #
    # divided_node_list = []
    # divided_way_list = []
    # divided_relation_list = []
    #
    # select_tags(divided_map_root, divided_node_list, divided_way_list, divided_relation_list)
    #
    # # Iterate on divided map and find missing members
    # for relation in divided_relation_list:
    #     relation_refs = [member for member in relation.iter("member")]
    #     for r in relation_refs:
    #         if r.attrib["ref"] not in [way.attrib["id"] for way in divided_way_list] and r.attrib["ref"] not in [
    #             node.attrib["id"] for node in divided_relation_list]:
    #             # find the way or relation in the whole map and add it to the divided map
    #             if r.attrib["type"] == "way":
    #                 for way in way_list:
    #                     if way.attrib["id"] == r.attrib["ref"]:
    #                         divided_map_root.append(way)
    #             elif r.attrib["type"] == "relation":
    #                 for relation in relation_list:
    #                     if relation.attrib["id"] == r.attrib["ref"]:
    #                         divided_map_root.append(relation)
    #             select_tags(divided_map_root, divided_node_list, divided_way_list, divided_relation_list)
    #
    # # Iterate on divided map's ways and find missing nodes
    # for way in divided_way_list:
    #     nd = [nd.attrib["ref"] for nd in way.iter("nd")]
    #     for n in nd:
    #         if n not in [node.attrib["id"] for node in divided_node_list]:
    #             # find the node in the whole map and add it to the divided map
    #             for node in node_list:
    #                 if node.attrib["id"] == n:
    #                     divided_map_root.append(node)
    #             select_tags(divided_map_root, divided_node_list, divided_way_list, divided_relation_list)
    #
    # divided_map_xml.write("/home/bzeren/projects/lanelet2-component/osmium/osmium-test/585431_new.osm")
