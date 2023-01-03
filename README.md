# Generate lanelet file

This package generates lanelet map from an edge image. It is a work in progress.

What it needs:
- An edge image (the result of inner region and lane image processed by `processed_lane`), which leaves only the edge of the visible road, plus long lane lines (which could be the median, or other lines not meant to be crossed)
- A route to indicate the direction of travel

The lanelet map file can be tested with `test_map` executable
