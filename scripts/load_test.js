// k6 load test for itinera-server
// Usage: k6 run scripts/load_test.js
import http from "k6/http";
import { check, sleep } from "k6";

const BASE_URL = __ENV.BASE_URL || "http://localhost:3002";

export const options = {
  stages: [
    { duration: "10s", target: 10 },
    { duration: "30s", target: 50 },
    { duration: "10s", target: 0 },
  ],
  thresholds: {
    http_req_duration: ["p(95)<1000"],
    http_req_failed: ["rate<0.01"],
  },
};

export default function () {
  // Health check
  const health = http.get(`${BASE_URL}/health`);
  check(health, { "health 200": (r) => r.status === 200 });

  // Route request
  const route = http.get(`${BASE_URL}/route?from=51.5074,-0.1278&to=51.5155,-0.1415`);
  check(route, { "route 200": (r) => r.status === 200 });

  // Nearest
  const nearest = http.get(`${BASE_URL}/nearest?lat=51.5074&lon=-0.1278`);
  check(nearest, { "nearest 200": (r) => r.status === 200 });

  // Isochrone
  const iso = http.get(`${BASE_URL}/isochrone?lat=51.5074&lon=-0.1278&time=600`);
  check(iso, { "isochrone 200": (r) => r.status === 200 });

  sleep(0.1);
}
