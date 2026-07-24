#pragma once

#include "baldr/graphconstants.h"
#include "midgard/pointll.h"

#include <boost/property_tree/ptree.hpp>

#include <string>
#include <vector>

namespace valhalla {
namespace mjolnir {

/**
 * Enriches unnamed pedestrian edges with the name of the nearest
 * road edge. For each unnamed pedestrian edge, finds the best matching
 * named road using spatial proximity (R-tree) refined by average polyline distance.
 *
 * Processes all local-level tiles in parallel using work-stealing (atomic tile counter)
 * with largest tiles scheduled first for load balancing.
 *
 * @param pt  Full configuration property tree (reads mjolnir.tile_dir, mjolnir.concurrency).
 */
void EnrichPedestrianEdgeNames(const boost::property_tree::ptree& pt);

// Internal helpers exposed for unit testing. Not part of the public API.
namespace detail {

// Maximum search distance in meters between a named road and a sidewalk.
constexpr float kMaxEnrichDistance = 50.f;

// Returns true if the edge `use` is a road-specific type (a name-giving candidate).
bool IsRoadUse(baldr::Use use);

// Returns true if the edge `use` is a pedestrian-specific type that should be enriched.
bool IsPedestrianUseToEnrich(baldr::Use use);

// Returns the point at 50% of the polyline's cumulative length.
// Falls back to the geometric middle vertex if trim_polyline fails.
midgard::PointLL PolylineMidpoint(const std::vector<midgard::PointLL>& shape);

// Computes a length-weighted average distance (in meters) from polyline 'from' to a set
// of polylines 'tos'. Each vertex of 'from' is projected onto the NEAREST of all 'tos',
// and every segment contributes the mean of its two endpoint distances, weighted by its
// share of the total length. Returns float::max() when the inputs are degenerate.
float AverageDistanceToPolylines(const std::vector<midgard::PointLL>& from,
                                 const std::vector<const std::vector<midgard::PointLL>*>& tos);

// A named road candidate: the street name and its full geometry.
struct NamedEdgeCandidate {
  std::string name;
  std::vector<midgard::PointLL> shape;
};

// Builds an R-tree over the given road candidates and returns the best matching street
// name for the pedestrian shape, or an empty string if none is within kMaxEnrichDistance.
// This wraps the same build+search logic used by the tile-build stage, over in-memory
// candidates, so the scoring can be unit-tested without tiles.
//
// @param candidates       named road candidates to index and score against
// @param pedestrian_shape  geometry of the unnamed pedestrian edge to name
// @param cos_lat           cos(latitude) used for the local cartesian projection
std::string FindNearestRoadName(const std::vector<NamedEdgeCandidate>& candidates,
                                const std::vector<midgard::PointLL>& pedestrian_shape,
                                float cos_lat);

} // namespace detail

} // namespace mjolnir
} // namespace valhalla
