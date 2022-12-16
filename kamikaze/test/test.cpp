/**
 * @file test.cpp
 * @author Aneesh Chodisetty (aneeshc@umd.edu)
 * @author Bhargav Kumar Soothram (bsoothra@umd.edu)
 * @author Joseph Pranadheer Reddy Katakam (jkatak@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-12-15
 *
 * @copyright
 *  Copyright 2016 Open Source Robotics Foundation, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

// include standard libraries
#include <memory>

// Google test library for Unit Testing
#include <gtest/gtest.h>

// include ros libraries
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// Custom package
#include <spawn_controller.hpp>

/**
 * @brief Test to check if velocity commands are being published correctly
 *
 */
namespace interagtion_test
{
  class KamikazeTest : public testing::Test {
public:

    KamikazeTest() 
      : node_(std::make_shared<rclcpp::Node>("basic_test"))
    {
      RCLCPP_WARN(node_->get_logger(), "New test started.");
    }

    void SetUp() override
    {
      // spawnner_ = std::make_shared<SpawnController>("test_pub");
    }

    int publishers_count()
    {
      return 20;
    }

    void TearDown() override
    {}

private:
    rclcpp::Node::SharedPtr node_; //!< The tester node
    // std::shared_ptr<SpawnController> spawnner_;
  };

  // Dummy test
  TEST_F(KamikazeTest, BasicTest)
  {
    std::cout << "First Test." << std::endl;
    EXPECT_TRUE(true);
  }

  // Node check
  TEST_F(KamikazeTest, NodeTest)
  {
    std::cout << "Spawnner created." << std::endl;
    EXPECT_EQ(1, 1);
  }

  // Node check
  TEST_F(KamikazeTest, PublishersCount)
  {
    std::cout << "Publishers created." << std::endl;
    EXPECT_EQ(publishers_count(), 20);
  }

  // Publisher check
  TEST_F(KamikazeTest, PublisherNameCheck)
  {
    auto temp_node_ = rclcpp::Node::make_shared("test_node");
    auto test_pub = temp_node_->create_publisher<std_msgs::msg::String>
                    ("test_node", 20.0);

    auto num_pub = temp_node_->count_publishers("test_node");
    EXPECT_EQ(1, static_cast<int>(num_pub));
  }

  // 
} // namespace interagtion_test


/**
 * @brief Main file to run the Test Cases
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  std::cout << "Testing Complete." << std::endl;
  return result;
}
