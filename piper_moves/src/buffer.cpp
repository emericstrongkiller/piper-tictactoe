// clustering with vote counting
// horizontal lines
std::vector<std::pair<float, int>> h_clusters; // (avg_position, vote_count)
int cluster_start = 0;
for (int i = 1; i <= static_cast<int>(h_lines.size()); i++) {
  if (i == static_cast<int>(h_lines.size()) ||
      h_lines[i] > h_lines[i - 1] + 10) {
    float cluster_sum = 0;
    int count = i - cluster_start;
    for (int j = cluster_start; j < i; j++) {
      cluster_sum += h_lines[j];
    }
    h_clusters.push_back({cluster_sum / count, count});
    cluster_start = i;
  }
}
// Sort by vote count (descending)
std::sort(h_clusters.begin(), h_clusters.end(),
          [](auto &a, auto &b) { return a.second > b.second; });

// Find best pair with valid spacing (60-120 for horizontal)
std::vector<float> clusterized_h_lines;
for (size_t i = 0; i < h_clusters.size() && clusterized_h_lines.size() < 2;
     i++) {
  for (size_t j = i + 1;
       j < h_clusters.size() && clusterized_h_lines.size() < 2; j++) {
    float spacing = std::abs(h_clusters[i].first - h_clusters[j].first);
    if (spacing >= 50 && spacing <= 200) {
      clusterized_h_lines = {h_clusters[i].first, h_clusters[j].first};
      break;
    }
  }
}
std::sort(clusterized_h_lines.begin(), clusterized_h_lines.end());

// vertical lines (same logic)
std::vector<std::pair<float, int>> v_clusters;
cluster_start = 0;
for (int i = 1; i <= static_cast<int>(v_lines.size()); i++) {
  if (i == static_cast<int>(v_lines.size()) ||
      v_lines[i] > v_lines[i - 1] + 20) {
    float cluster_sum = 0;
    int count = i - cluster_start;
    for (int j = cluster_start; j < i; j++) {
      cluster_sum += v_lines[j];
    }
    v_clusters.push_back({cluster_sum / count, count});
    cluster_start = i;
  }
}
std::sort(v_clusters.begin(), v_clusters.end(),
          [](auto &a, auto &b) { return a.second > b.second; });

// Find best pair with valid spacing (70-130 for vertical)
std::vector<float> clusterized_v_lines;
for (size_t i = 0; i < v_clusters.size() && clusterized_v_lines.size() < 2;
     i++) {
  for (size_t j = i + 1;
       j < v_clusters.size() && clusterized_v_lines.size() < 2; j++) {
    float spacing = std::abs(v_clusters[i].first - v_clusters[j].first);
    if (spacing >= 50 && spacing <= 300) {
      clusterized_v_lines = {v_clusters[i].first, v_clusters[j].first};
      break;
    }
  }
}
std::sort(clusterized_v_lines.begin(), clusterized_v_lines.end());

print_grid_lines(clusterized_h_lines, clusterized_v_lines, immage);
std::vector<std::vector<float>> ret = {clusterized_h_lines,
                                       clusterized_v_lines};
return ret;