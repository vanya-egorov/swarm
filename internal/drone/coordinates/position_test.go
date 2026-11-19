package coordinates

import (
	"math"
	"testing"
)

func TestVillageSITLLaunchDistance(t *testing.T) {
	// GPS measured after reloading village.wbt at Webots north=225, east=-290 m.
	got := HorizontalDistance(-35.363261, 149.165237, -35.3612398, 149.1620426)
	want := math.Hypot(225, 290)
	if math.Abs(got-want) > 0.03 {
		t.Fatalf("village launch distance = %.3f m, want %.3f m within GPS quantization", got, want)
	}
}
