#include "baldr/graphreader.h"
#include "gurka.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using namespace valhalla;

namespace {

// Reads the street names of the edge between two named nodes.
std::vector<std::string> edge_names(baldr::GraphReader& reader,
                                    const gurka::nodelayout& nodes,
                                    const std::string& from,
                                    const std::string& to) {
  auto [edge_id, edge] = gurka::findEdgeByNodes(reader, nodes, from, to);
  auto tile = reader.GetGraphTile(edge_id);
  return tile->edgeinfo(edge).GetNames();
}

// Builds tiles with the pedestrian-name-enrichment stage optionally enabled.
gurka::map build(const gurka::ways& ways,
                 const std::string& workdir,
                 bool enrich,
                 const std::string& ascii_map,
                 double gridsize = 10) {
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  std::unordered_map<std::string, std::string> config{{"mjolnir.concurrency", "1"}};
  if (enrich) {
    config["mjolnir.enrich_pedestrian_names"] = "true";
  }
  return gurka::buildtiles(layout, ways, {}, {}, workdir, config);
}

} // namespace

// A named road with an unnamed parallel sidewalk ~20 m away, connected at the ends.
//
//   A--------------B    <- AB: named road ("Main Street")
//   |              |
//   C--------------D    <- CD: unnamed sidewalk (candidate for enrichment)
//
constexpr const char* kParallelMap = R"(
    A--------------B
    |              |
    C--------------D
  )";

const gurka::ways kParallelWays = {
    {"AB", {{"highway", "residential"}, {"name", "Main Street"}}},
    {"AC", {{"highway", "footway"}, {"footway", "sidewalk"}, {"name", ""}}},
    {"BD", {{"highway", "footway"}, {"footway", "sidewalk"}, {"name", ""}}},
    {"CD", {{"highway", "footway"}, {"footway", "sidewalk"}, {"name", ""}}},
};

TEST(PedestrianNameEnrichment, EnrichesUnnamedSidewalk) {
  auto map = build(kParallelWays, "test/data/ped_enrich_on", true, kParallelMap);
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  auto names = edge_names(reader, map.nodes, "C", "D");
  ASSERT_EQ(names.size(), 1);
  EXPECT_EQ(names[0], "Main Street");
}

TEST(PedestrianNameEnrichment, DisabledByDefault) {
  auto map = build(kParallelWays, "test/data/ped_enrich_off", false, kParallelMap);
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  // Without the flag the sidewalk keeps no name.
  EXPECT_TRUE(edge_names(reader, map.nodes, "C", "D").empty());
}

TEST(PedestrianNameEnrichment, OnlySidewalkUseIsEnriched) {
  // A plain footway (not a sidewalk) sitting right next to the road must NOT be enriched.
  const char* ascii = R"(
    A--------------B
    |
    C
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}, {"name", "Main Street"}}},
      {"AC", {{"highway", "footway"}, {"name", ""}}}, // footway, use != kSidewalk
  };
  auto map = build(ways, "test/data/ped_enrich_footway", true, ascii);
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  EXPECT_TRUE(edge_names(reader, map.nodes, "A", "C").empty());
}

TEST(PedestrianNameEnrichment, PreservesExistingSidewalkName) {
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}, {"name", "Main Street"}}},
      {"AC", {{"highway", "footway"}, {"footway", "sidewalk"}, {"name", ""}}},
      {"BD", {{"highway", "footway"}, {"footway", "sidewalk"}, {"name", ""}}},
      {"CD", {{"highway", "footway"}, {"footway", "sidewalk"}, {"name", "Historic Promenade"}}},
  };
  auto map = build(ways, "test/data/ped_enrich_named", true, kParallelMap);
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  auto names = edge_names(reader, map.nodes, "C", "D");
  ASSERT_EQ(names.size(), 1);
  EXPECT_EQ(names[0], "Historic Promenade");
}

TEST(PedestrianNameEnrichment, NoEnrichmentBeyondThreshold) {
  // The sidewalk CD is ~60 m from the only road AB (> 50 m max), so it stays unnamed.
  const char* ascii = R"(
    A--------------B
    |
    |
    |
    |
    |
    |
    C--------------D
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}, {"name", "Main Street"}}},
      {"AC", {{"highway", "footway"}, {"footway", "sidewalk"}, {"name", ""}}},
      {"CD", {{"highway", "footway"}, {"footway", "sidewalk"}, {"name", ""}}},
  };
  auto map = build(ways, "test/data/ped_enrich_far", true, ascii);
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  EXPECT_TRUE(edge_names(reader, map.nodes, "C", "D").empty());
}

TEST(PedestrianNameEnrichment, NearestRoadWins) {
  // CD sidewalk sits ~20 m below "Near Street" (AB) and ~60 m above "Far Street" (EF).
  //
  //   A--------------B   Near Street
  //   |              |
  //   C--------------D   unnamed sidewalk
  //   |              |
  //   |              |
  //   |              |
  //   E--------------F   Far Street
  //
  const char* ascii = R"(
    A--------------B
    |              |
    C--------------D
    |              |
    |              |
    |              |
    E--------------F
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}, {"name", "Near Street"}}},
      {"EF", {{"highway", "residential"}, {"name", "Far Street"}}},
      {"AC", {{"highway", "footway"}, {"footway", "sidewalk"}, {"name", ""}}},
      {"BD", {{"highway", "footway"}, {"footway", "sidewalk"}, {"name", ""}}},
      {"CD", {{"highway", "footway"}, {"footway", "sidewalk"}, {"name", ""}}},
      {"CE", {{"highway", "footway"}, {"footway", "sidewalk"}, {"name", ""}}},
      {"DF", {{"highway", "footway"}, {"footway", "sidewalk"}, {"name", ""}}},
  };
  auto map = build(ways, "test/data/ped_enrich_nearest", true, ascii);
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  auto names = edge_names(reader, map.nodes, "C", "D");
  ASSERT_EQ(names.size(), 1);
  EXPECT_EQ(names[0], "Near Street");
}

TEST(PedestrianNameEnrichment, EnrichedNameDrivesNarrative) {
  // The whole point of the feature: pedestrian guidance should mention the street name
  // instead of a generic "the walkway".
  auto map = build(kParallelWays, "test/data/ped_enrich_narrative", true, kParallelMap);
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "D"}, "pedestrian");

  ASSERT_EQ(result.directions().routes_size(), 1);
  const auto& maneuvers = result.directions().routes(0).legs(0).maneuver();
  ASSERT_GT(maneuvers.size(), 0);
  EXPECT_NE(maneuvers.Get(0).text_instruction().find("Main Street"), std::string::npos)
      << "Instruction was: " << maneuvers.Get(0).text_instruction();
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
