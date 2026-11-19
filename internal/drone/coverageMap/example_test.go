package coverageMap_test

import (
	"fmt"

	"github.com/vanya-egorov/swarm/internal/drone/coverageMap"
)

func ExamplePlan() {
	area := coverageMap.Area{Max: coverageMap.Point{NorthM: 10, EastM: 6}}
	config := coverageMap.DefaultConfig()
	config.SwathWidthM, config.LaneSpacingM = 2, 2
	assignments, err := coverageMap.Plan(area, []uint8{1, 2, 3}, config)
	if err != nil {
		panic(err)
	}
	for _, assignment := range assignments {
		fmt.Printf("ID %d: east %.0f..%.0f m, %d waypoints\n",
			assignment.SystemID, assignment.Area.Min.EastM, assignment.Area.Max.EastM, len(assignment.Waypoints))
	}
	// Output:
	// ID 1: east 0..2 m, 2 waypoints
	// ID 2: east 2..4 m, 2 waypoints
	// ID 3: east 4..6 m, 2 waypoints
}
