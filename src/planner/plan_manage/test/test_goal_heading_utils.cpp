#include <gtest/gtest.h>
#include <limits>
#include <plan_manage/goal_heading_utils.h>
#include <plan_manage/reference_path_utils.h>

using namespace scan_planner;

TEST(GoalHeading, IdentityAndBothQuaternionSignsAreExplicitZero)
{
  geometry_msgs::msg::Quaternion q;
  q.w = 1.0;
  ASSERT_TRUE(terminalYawFromQuaternion(q));
  EXPECT_DOUBLE_EQ(*terminalYawFromQuaternion(q), 0.0);
  q.w = -1.0;
  EXPECT_DOUBLE_EQ(*terminalYawFromQuaternion(q), 0.0);
}

TEST(GoalHeading, NormalizesScaledQuaternionsAndPreservesWrap)
{
  for (double yaw : {0.0, 0.4, -0.4, M_PI/2, -M_PI/2, M_PI, -M_PI,
                     179.0*M_PI/180, -179.0*M_PI/180})
    for (double scale : {1.0, -2.0, 1e300})
    {
      geometry_msgs::msg::Quaternion q;
      q.z = scale*std::sin(yaw/2); q.w = scale*std::cos(yaw/2);
      auto actual = terminalYawFromQuaternion(q);
      ASSERT_TRUE(actual);
      EXPECT_NEAR(std::remainder(*actual-yaw, 2*M_PI), 0.0, 1e-12);
    }
}

TEST(GoalHeading, RejectsAbsentAndNonFiniteOrientation)
{
  geometry_msgs::msg::Quaternion q;
  q.w = 0.0; // ROS initializes a default Quaternion as the valid identity.
  EXPECT_FALSE(terminalYawFromQuaternion(q));
  q.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(terminalYawFromQuaternion(q));
  q.w = 1.0; q.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(terminalYawFromQuaternion(q));
}

TEST(GoalHeading, PathSimplificationAndBodyHeightKeepFinalHeading)
{
  for (double yaw : {0.0, M_PI/2, -M_PI/2, M_PI})
  {
    nav_msgs::msg::Path path;
    for (int i=0; i<11; ++i)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = i*0.1;
      pose.pose.position.z = -0.5;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path.poses.back().pose.orientation.z = std::sin(yaw/2);
    path.poses.back().pose.orientation.w = std::cos(yaw/2);
    std::vector<Eigen::Vector3d> waypoints;
    std::optional<double> actual;
    ASSERT_TRUE(prepareReferenceWaypoints(path, 0.4, 0.3, waypoints,
                                          nullptr, 0.01, &actual));
    ASSERT_TRUE(actual);
    EXPECT_NEAR(std::remainder(*actual-yaw, 2*M_PI), 0.0, 1e-12);
    EXPECT_EQ(waypoints.size(), 2u);
    EXPECT_DOUBLE_EQ(waypoints.back().x(), 1.0);
    EXPECT_NEAR(waypoints.back().z(), -0.1, 1e-12);
    scan_planner_msgs::msg::Bspline message;
    encodeTerminalGoalYaw(message, actual, 2.0);
    ASSERT_TRUE(decodeTerminalGoalYaw(message));
    EXPECT_NEAR(std::remainder(*decodeTerminalGoalYaw(message)-yaw, 2*M_PI), 0.0, 1e-12);
  }
}

TEST(GoalHeading, EveryLocalWindowKeepsGlobalHeadingAndStopsClearIt)
{
  scan_planner_msgs::msg::Bspline message;
  for (double duration : {0.1, 2.0, 13.0})
  {
    encodeTerminalGoalYaw(message, -M_PI/2, duration);
    ASSERT_EQ(message.yaw_pts.size(), 2u);
    EXPECT_DOUBLE_EQ(message.yaw_dt, duration);
    EXPECT_DOUBLE_EQ(*decodeTerminalGoalYaw(message), -M_PI/2);
  }
  encodeTerminalGoalYaw(message, std::nullopt, 3.0);
  EXPECT_TRUE(message.yaw_pts.empty());
  EXPECT_FALSE(decodeTerminalGoalYaw(message));
  encodeTerminalGoalYaw(message, 0.0, 3.0);
  ASSERT_TRUE(decodeTerminalGoalYaw(message));
  EXPECT_DOUBLE_EQ(*decodeTerminalGoalYaw(message), 0.0);
  for (double duration : {0.0, -1.0, std::numeric_limits<double>::infinity()})
  {
    encodeTerminalGoalYaw(message, 0.0, duration);
    EXPECT_FALSE(decodeTerminalGoalYaw(message));
  }
}

TEST(GoalHeading, MalformedProfileCannotRetainPreviousGoal)
{
  scan_planner_msgs::msg::Bspline message;
  encodeTerminalGoalYaw(message, 1.0, 3.0);
  message.yaw_pts[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(decodeTerminalGoalYaw(message));
  message.yaw_pts = {1.0};
  EXPECT_FALSE(decodeTerminalGoalYaw(message));
  message.yaw_pts = {1.0, 1.0}; message.yaw_dt = 0.0;
  EXPECT_FALSE(decodeTerminalGoalYaw(message));
  nav_msgs::msg::Path empty;
  std::vector<Eigen::Vector3d> waypoints;
  std::optional<double> previous = 1.0;
  EXPECT_FALSE(prepareReferenceWaypoints(empty, 0.4, 0.1, waypoints,
                                        nullptr, 0.0, &previous));
  EXPECT_FALSE(previous);
}

TEST(TerminalAlignment, MovingAndEmergencyTrajectoriesCannotRequestRotation)
{
  scan_planner_msgs::msg::Bspline message;
  geometry_msgs::msg::Point p;
  p.x = 1.0; p.y = 2.0;
  message.pos_pts = {p,p,p,p,p,p};
  EXPECT_FALSE(isTerminalAlignmentTrajectory(message));
  encodeTerminalGoalYaw(message, M_PI/2, 3.0);
  EXPECT_TRUE(isTerminalAlignmentTrajectory(message));
  message.pos_pts.back().x += 0.01;
  EXPECT_FALSE(isTerminalAlignmentTrajectory(message));
  message.pos_pts.back() = p;
  message.pos_pts.back().z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(isTerminalAlignmentTrajectory(message));
  // Ordinary stop supersedes the previous rotation request.
  message.pos_pts.back() = p;
  encodeTerminalGoalYaw(message, std::nullopt, 3.0);
  EXPECT_FALSE(isTerminalAlignmentTrajectory(message));
}

TEST(TerminalAlignment, RealHeadingAndSettlingMustAgreeAcrossAngleWrap)
{
  EXPECT_FALSE(terminalHeadingReached(M_PI/2, 0.0, 0.1));
  EXPECT_FALSE(terminalHeadingReached(M_PI/2, std::nullopt, 0.1));
  EXPECT_TRUE(terminalHeadingReached(M_PI/2, M_PI/2-0.09, 0.1));
  EXPECT_TRUE(terminalHeadingReached(M_PI, -M_PI, 0.1));
  EXPECT_TRUE(terminalHeadingReached(179*M_PI/180, -179*M_PI/180, 0.1));
  EXPECT_TRUE(terminalHeadingReached(0.0, 0.0, 0.1));
  EXPECT_TRUE(terminalHeadingReached(std::nullopt, 0.0, 0.1));
}

TEST(TerminalAlignment, MinimumEffectiveRateAndShortestTurnStopAtTolerance)
{
  EXPECT_DOUBLE_EQ(terminalAlignmentYawRate(0.15,0.1,1.5,0.35,0.35),0.35);
  EXPECT_DOUBLE_EQ(terminalAlignmentYawRate(-0.15,0.1,1.5,0.35,0.35),-0.35);
  EXPECT_DOUBLE_EQ(terminalAlignmentYawRate(0.09,0.1,1.5,0.35,0.35),0.0);
  EXPECT_DOUBLE_EQ(terminalAlignmentYawRate(2*M_PI-0.15,0.1,1.5,0.35,0.35),-0.35);
  EXPECT_DOUBLE_EQ(terminalAlignmentYawRate(std::numeric_limits<double>::infinity(),
                                            0.1,1.5,0.35,0.35),0.0);
}
