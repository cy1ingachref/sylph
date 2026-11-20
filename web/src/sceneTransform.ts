// ONE documented geographic -> local scene transform. Both the route and the
// drone use this function, so they cannot drift apart. Local ENU approximation
// (same range limits as sylph.mission.lawnmower_pattern: small fields, not polar).
export interface LocalPoint {
  x: number
  z: number
}

export function makeTransform(originLat: number, originLon: number) {
  const M_PER_DEG_LAT = 111_320
  const mPerDegLon = 111_320 * Math.cos((originLat * Math.PI) / 180)
  return (latDeg: number, lonDeg: number): LocalPoint => ({
    x: (lonDeg - originLon) * mPerDegLon,
    // Three.js ground plane: north = -Z
    z: -(latDeg - originLat) * M_PER_DEG_LAT,
  })
}
