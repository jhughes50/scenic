/*!
* @Author Jason Hughes
* @Date January 2026
*
* @About Export the graph
*/

#include <json/json.h>
#include <fstream>
#include <iostream>

namespace Scenic
{
struct Exporter
{
    Exporter() = default;
    static void exportToJson(std::shared_ptr<Graph> graph);
};

inline void Exporter::exportToJson(std::shared_ptr<Graph> graph)
{
    Json::Value root;
    
    Json::Vake origin(Json::arrayValue);
    origin.append(origin_.easting);
    origin.append(origin_.northing);

    // export to the nodes
    Json::Value regions(Json::arrayValue);
    Json::Value objects(Json::arrayValue);
    for (const auto& [nid, node] : graph->getNodes()) {
        Json::Value entry;
        // make the label
        std::string label = texts_[node->getClassLabel()];
        label += "_"+static_cast<std::string>(nid);
        entry["name"] = label;

        Json::Value coords(Json::arrayValue);
        UTMPoint point = node->getUtmCoordinate();
        coords.append(point.easting);
        coords.append(point.northing);
        entry["coords"] = coords;

        if (node->getNodeLevel() == GraphLevel::REGION) {
            regions.append(entry);
        } else if (node->getNodeLevel() == GraphLevel::OBJECT) {
            objects.append(entry);
        } else {
            LOG(ERROR) << "A node with no GraphLevel was found during export, NID: " << nid;
        }
    }
    root["objects"] = objects;
    root["regions"] = regions;

    Json::Value object_connections(Json::arrayValue);
    Json::Value region_connections(Json::arrayValue);
    for (const auto& [nids, edge] : graph->getEdges()) {
        std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>> nodes = edge->getNodePair();

        std::string parent_label = texts_[nodes.first->getClassLabel()];
        parent_label += "_"+static_cast<std::string>(nids.first);

        std::string child_label = texts_[nodes.second->getClassLabel()];
        child_label += "_"+static_cast<std::string>(nids.second);

        Json::Value entry(Json::arrayValue);
        entry.append(parent_label);
        entry.append(child_label);

        if (nodes.first->getNodeLevel() == GraphLevel::REGION && nodes.second->getNodeLevel() == GraphLevel::REGION) {
            region_connections.append(entry); 
        } else if (nodes.first->getNodeLevel() == GraphLevel::OBJECT || nodes.second->getNodeLevel() == GraphLevel::OBJECT) {
            object_connections.append(entry);
        }
    }
    root["object_connections"] = object_connections;
    root["region_connections"] = region_connections;
}

}
