#!/bin/sh
set -e

# If OSM file exists and graph hasn't been built yet, import it
if [ -f /data/region.osm.pbf ] && [ ! -f /data/graph.bin ]; then
    echo "Building routing graph from /data/region.osm.pbf..."
    itinera import --input /data/region.osm.pbf --output /data/graph.bin
    echo "Graph built successfully."
fi

# If no graph file exists at all, warn and exit gracefully
if [ ! -f /data/graph.bin ]; then
    echo "WARNING: No graph file at /data/graph.bin"
    echo "Place an OSM extract at /data/region.osm.pbf and restart, or run:"
    echo "  itinera import --input /path/to/extract.osm.pbf --output /data/graph.bin"
    echo "Sleeping to keep container alive for debugging..."
    exec sleep infinity
fi

exec itinera "$@"
