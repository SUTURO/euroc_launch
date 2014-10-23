/*
 * TestNode.cpp
 *
 *  Created on: Oct 23, 2014
 *      Author: moritz
 */

#include <ros/ros.h>
#include <suturo_msgs/Task.h>
#include <gztest/test_client.h>

class TestNode
{
public:
  TestNode();
  virtual ~TestNode();
  void startListening();
  void check();
private:
  void parseCallback(const suturo_msgs::Task &description);
  void checkTargetZones(const std::vector<suturo_msgs::TargetZone> &targetZones);
  void checkObjectsGrasped(const std::vector<suturo_msgs::TargetZone> &targetZones);
  suturo_msgs::Task description;
  gztest::TestClient client;
  int points;
};

TestNode::TestNode() :
    client("http://localhost:8080")
{
  points = 0;
}

void TestNode::parseCallback(const suturo_msgs::Task &description)
{
  this->description = description;
}

void TestNode::startListening()
{
  client.MonitorLinkEvents("lwr", "gripper_fixed_on_lwr", "base");
  printf("Registered link event monitor\n");
}

void TestNode::check()
{
  ros::NodeHandle nh;
  ros::Subscriber parser = nh.subscribe("/suturo/yaml_pars0r", 1, &TestNode::parseCallback, this);
  while (true)
  {
    ros::spinOnce();
    if (this->description.task_name.length() == 0)
    {
      sleep(1);
      continue;
    }
    break;
  }
  printf("Received task description for task: %s\n", this->description.task_name.c_str());
  checkTargetZones(this->description.target_zones);
  checkObjectsGrasped(this->description.target_zones);
  printf("Total number of reached points: %d\n", points);
}

void TestNode::checkTargetZones(const std::vector<suturo_msgs::TargetZone> &targetZones)
{
  for (std::vector<suturo_msgs::TargetZone>::const_iterator targetZone = targetZones.begin();
      targetZone != targetZones.end(); ++targetZone)
  {
    std::vector<double> pos = client.GetPosition("obj:" + (*targetZone).expected_object);
    double dx = pos[0] - (*targetZone).target_position.x;
    double dy = pos[1] - (*targetZone).target_position.y;
    double dist = sqrt(dx * dx + dy * dy);
    if (dist < (*targetZone).max_distance)
    {
      printf("SUCCESS: %s is on its target zone!\n", (*targetZone).expected_object.c_str());
      points += 5;
    }
    else
    {
      printf("FAIL: %s is NOT on its target zone!\n", (*targetZone).expected_object.c_str());
    }
  }
}

void TestNode::checkObjectsGrasped(const std::vector<suturo_msgs::TargetZone> &targetZones)
{
  std::vector<WatcherEvent> list = client.GetLinkEventHistory("lwr", "gripper_fixed_on_lwr", "base");
  for (std::vector<suturo_msgs::TargetZone>::const_iterator targetZone = targetZones.begin();
      targetZone != targetZones.end(); ++targetZone)
  {
    bool grasped = false;
    for (std::vector<WatcherEvent>::iterator entry = list.begin(); entry != list.end(); ++entry)
    {
      grasped |= (*entry).tail.tail.head == "obj:" + (*targetZone).expected_object && (*entry).tail.head;
    }
    if (grasped)
    {
      printf("SUCCESS: %s was grasped!\n", (*targetZone).expected_object.c_str());
      points += 5;
    } else {
      printf("FAIL: %s was NOT grasped!\n", (*targetZone).expected_object.c_str());
    }

  }
}

TestNode::~TestNode()
{
}

int main(int _argc, char** _argv)
{
  printf("TestPlugin client for EuRoC challenge 2 simulator\n\n");
  if (_argc <= 1)
  {
    printf("Usage: TestNode --init # before task\n       TestNode --check # after task\n");
    exit(EXIT_FAILURE);
  }
  ros::init(_argc, _argv, "TestNode");
  TestNode node;
  if (strcmp(_argv[1], "--init") == 0)
  {
    node.startListening();
  }
  else if (strcmp(_argv[1], "--check") == 0)
  {
    node.check();
  }
  exit(EXIT_SUCCESS);
}

