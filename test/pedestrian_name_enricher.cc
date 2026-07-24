#include "mjolnir/pedestrian_name_enricher.h"
#include "baldr/graphconstants.h"
#include "midgard/pointll.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir::detail;

namespace {

// Base location for the synthetic geometry (a mid-latitude so the cartesian
// projection factor is clearly < 1).
constexpr double kBaseLat = 45.0;
constexpr double kBaseLng = 5.0;
const float kCosLat = static_cast<float>(std::cos(kBaseLat * M_PI / 180.0));

// Meters-per-degree helpers around kBaseLat (good enough for building test shapes).
constexpr double kMetersPerDegLat = 111320.0;
double MetersPerDegLng() {
  return kMetersPerDegLat * std::cos(kBaseLat * M_PI / 180.0);
}
double DegLatForMeters(double m) {
  return m / kMetersPerDegLat;
}
double DegLngForMeters(double m) {
  return m / MetersPerDegLng();
}

// Horizontal segment at a given north offset (meters) spanning [lng0_m, lng1_m] (meters east
// of kBaseLng), returned as a polyline of `n` evenly spaced vertices.
std::vector<PointLL>
HorizontalShape(double north_offset_m, double lng0_m, double lng1_m, size_t n = 2) {
  std::vector<PointLL> shape;
  const double lat = kBaseLat + DegLatForMeters(north_offset_m);
  for (size_t i = 0; i < n; ++i) {
    const double t = n == 1 ? 0.0 : static_cast<double>(i) / (n - 1);
    const double lng_m = lng0_m + t * (lng1_m - lng0_m);
    shape.emplace_back(kBaseLng + DegLngForMeters(lng_m), lat);
  }
  return shape;
}

} // namespace

/*************************** IsRoadUse / IsPedestrianUseToEnrich ***************************/

TEST(PedestrianNameEnricher, IsRoadUseTruthTable) {
  // Road-like uses are name-giving candidates.
  for (auto use : {Use::kRoad, Use::kRamp, Use::kTurnChannel, Use::kTrack, Use::kDriveway,
                   Use::kAlley, Use::kParkingAisle, Use::kEmergencyAccess, Use::kDriveThru,
                   Use::kCuldesac, Use::kLivingStreet, Use::kServiceRoad}) {
    EXPECT_TRUE(IsRoadUse(use)) << "use=" << static_cast<int>(use);
  }
  // Pedestrian / non-road uses are not candidates.
  for (auto use :
       {Use::kSidewalk, Use::kFootway, Use::kPath, Use::kSteps, Use::kPedestrian, Use::kCycleway}) {
    EXPECT_FALSE(IsRoadUse(use)) << "use=" << static_cast<int>(use);
  }
}

TEST(PedestrianNameEnricher, IsPedestrianUseToEnrichOnlySidewalk) {
  EXPECT_TRUE(IsPedestrianUseToEnrich(Use::kSidewalk));
  // Everything else (including other pedestrian uses) must be left untouched.
  for (auto use : {Use::kFootway, Use::kPath, Use::kSteps, Use::kPedestrian, Use::kPedestrianCrossing,
                   Use::kRoad, Use::kCycleway, Use::kTrack}) {
    EXPECT_FALSE(IsPedestrianUseToEnrich(use)) << "use=" << static_cast<int>(use);
  }
}

/*************************** PolylineMidpoint ***************************/

TEST(PedestrianNameEnricher, PolylineMidpointIsHalfCumulativeLength) {
  // Asymmetric polyline: a long first segment then a short one. The midpoint by cumulative
  // length falls inside the long first segment, NOT at the geometric middle vertex B.
  auto shape = HorizontalShape(0.0, 0.0, 0.0, 1); // placeholder
  shape.clear();
  shape.emplace_back(kBaseLng + DegLngForMeters(0.0), kBaseLat);   // A
  shape.emplace_back(kBaseLng + DegLngForMeters(300.0), kBaseLat); // B (300 m east)
  shape.emplace_back(kBaseLng + DegLngForMeters(360.0), kBaseLat); // C (60 m further)

  PointLL mid = PolylineMidpoint(shape);

  // Total length 360 m -> 50% is 180 m east of A, well before B (300 m).
  const double expected_lng = kBaseLng + DegLngForMeters(180.0);
  EXPECT_NEAR(mid.lng(), expected_lng, DegLngForMeters(5.0));
  EXPECT_NEAR(mid.lat(), kBaseLat, DegLatForMeters(0.5));
  EXPECT_LT(mid.lng(), shape[1].lng()); // strictly before vertex B
}

TEST(PedestrianNameEnricher, PolylineMidpointTwoPoints) {
  std::vector<PointLL> shape = {{kBaseLng, kBaseLat}, {kBaseLng + DegLngForMeters(100.0), kBaseLat}};
  PointLL mid = PolylineMidpoint(shape);
  EXPECT_NEAR(mid.lng(), kBaseLng + DegLngForMeters(50.0), DegLngForMeters(2.0));
}

/*************************** AverageDistanceToPolylines ***************************/

TEST(PedestrianNameEnricher, AverageDistanceDegenerateInputs) {
  const float kMax = std::numeric_limits<float>::max();
  std::vector<PointLL> single = {{kBaseLng, kBaseLat}};
  std::vector<PointLL> line = HorizontalShape(0.0, 0.0, 100.0);

  // 'from' with fewer than 2 vertices -> max
  EXPECT_EQ(AverageDistanceToPolylines(single, {&line}), kMax);
  // empty 'tos' -> max
  EXPECT_EQ(AverageDistanceToPolylines(line, {}), kMax);
}

TEST(PedestrianNameEnricher, AverageDistanceParallelLine) {
  // A pedestrian line 11 m north of a parallel road: score should be ~11 m.
  const double offset_m = 11.0;
  std::vector<PointLL> from = HorizontalShape(offset_m, 0.0, 200.0, 5);
  std::vector<PointLL> to = HorizontalShape(0.0, -50.0, 250.0, 5);

  float score = AverageDistanceToPolylines(from, {&to});
  EXPECT_NEAR(score, offset_m, 2.0f);
}

TEST(PedestrianNameEnricher, AverageDistanceIsLengthWeighted) {
  // 'from' = A--B (long, ~11 m from 'to') then B--C (short, C is far ~44 m from 'to').
  // A length-weighted distance is dominated by the long near segment, so the score stays
  // well below what a max/Hausdorff distance (~44 m) would give.
  std::vector<PointLL> from;
  from.emplace_back(kBaseLng + DegLngForMeters(0.0), kBaseLat + DegLatForMeters(11.0));   // A
  from.emplace_back(kBaseLng + DegLngForMeters(200.0), kBaseLat + DegLatForMeters(11.0)); // B
  from.emplace_back(kBaseLng + DegLngForMeters(210.0), kBaseLat + DegLatForMeters(44.0)); // C (far)

  std::vector<PointLL> to = HorizontalShape(0.0, -50.0, 260.0, 6);

  float score = AverageDistanceToPolylines(from, {&to});
  EXPECT_GT(score, 11.0f);
  EXPECT_LT(score, 25.0f); // far vertex C barely moves it
}

/*************************** FindNearestRoadName ***************************/

TEST(PedestrianNameEnricher, FindNearestRoadNameBasic) {
  std::vector<NamedEdgeCandidate> candidates = {
      {"Main Street", HorizontalShape(11.0, -50.0, 250.0, 4)},
  };
  std::vector<PointLL> sidewalk = HorizontalShape(0.0, 0.0, 200.0, 4);

  EXPECT_EQ(FindNearestRoadName(candidates, sidewalk, kCosLat), "Main Street");
}

TEST(PedestrianNameEnricher, FindNearestRoadNameBeyondThreshold) {
  // Only candidate is ~111 m away, beyond kMaxEnrichDistance (50 m) -> no name.
  std::vector<NamedEdgeCandidate> candidates = {
      {"Far Street", HorizontalShape(111.0, -50.0, 250.0, 4)},
  };
  std::vector<PointLL> sidewalk = HorizontalShape(0.0, 0.0, 200.0, 4);

  EXPECT_TRUE(FindNearestRoadName(candidates, sidewalk, kCosLat).empty());
}

TEST(PedestrianNameEnricher, FindNearestRoadNamePicksClosest) {
  std::vector<NamedEdgeCandidate> candidates = {
      {"Near Street", HorizontalShape(10.0, -50.0, 250.0, 4)},
      {"Other Street", HorizontalShape(35.0, -50.0, 250.0, 4)},
  };
  std::vector<PointLL> sidewalk = HorizontalShape(0.0, 0.0, 200.0, 4);

  EXPECT_EQ(FindNearestRoadName(candidates, sidewalk, kCosLat), "Near Street");
}

TEST(PedestrianNameEnricher, FindNearestRoadNameGroupsSplitRoadByName) {
  // A long sidewalk (0..400 m) parallel to a street split into two same-named edges, each
  // covering half its length at ~15 m. A rival "Short Street" covers only the first 60 m at a
  // closer ~8 m. Grouping by name must let the long street win: "Short Street" is far from the
  // rest of the sidewalk, so its length-weighted average is large.
  std::vector<NamedEdgeCandidate> candidates = {
      {"Long Avenue", HorizontalShape(15.0, -20.0, 210.0, 4)},
      {"Long Avenue", HorizontalShape(15.0, 190.0, 420.0, 4)},
      {"Short Street", HorizontalShape(8.0, -10.0, 60.0, 3)},
  };
  std::vector<PointLL> sidewalk = HorizontalShape(0.0, 0.0, 400.0, 9);

  EXPECT_EQ(FindNearestRoadName(candidates, sidewalk, kCosLat), "Long Avenue");
}

TEST(PedestrianNameEnricher, FindNearestRoadNameNoCandidates) {
  std::vector<PointLL> sidewalk = HorizontalShape(0.0, 0.0, 200.0, 4);
  EXPECT_TRUE(FindNearestRoadName({}, sidewalk, kCosLat).empty());
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
