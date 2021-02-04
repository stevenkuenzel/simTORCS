package simtorcs.track

class ConnectingSegment(from: Segment, to: Segment, track: Track) : Segment(from, track) {
    init {
        p1 = from.p3
        p2 = from.p4
        p3 = to.p1
        p4 = to.p2

        updateSegmentLines()
        updateCentres()
    }
}