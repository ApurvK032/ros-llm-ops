// Persistent transport connection: visualization never waits inside the robot controller.
#include <iostream>
#include <string>
#include <google/protobuf/text_format.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/marker_v.pb.h>
#include <gz/transport/Node.hh>

int main() {
  gz::transport::Node node;
  std::string line;
  while (std::getline(std::cin, line)) {
    gz::msgs::Marker_V markers;
    gz::msgs::Boolean reply;
    bool result = false;
    const bool parsed = google::protobuf::TextFormat::ParseFromString(line, &markers);
    const bool received = parsed && node.Request("/marker_array", markers, 300, reply, result);
    std::cout << ((received && result && reply.data()) ? "ok" : received ? "rejected" : "unavailable") << std::endl;
  }
}
