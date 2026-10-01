package sendspin

import (
	"math/rand"
	"testing"
	"time"
)

// A DAC off nominal, measured through ±2ms of noise (pessimistic for DMA
// pointer granularity): the smoothed estimate settles to a few hundred µs,
// against a 1ms sync floor.
func TestOutputClockRecoversTheDACThroughJitter(t *testing.T) {
	period := time.Duration(2048) * time.Second / 48000
	for _, ppm := range []float64{-2000, -560, 0, 345, 2000} {
		c := NewOutputClock(period)
		rng := rand.New(rand.NewSource(89))
		truePer := float64(period) * (1 - ppm/1e6)
		t0 := epoch.Add(time.Hour)
		worst := time.Duration(0)
		for k := 0; k < 2000; k++ {
			truth := t0.Add(time.Duration(float64(k) * truePer))
			jitter := time.Duration((rng.Float64()*2 - 1) * float64(2*time.Millisecond))
			got := c.Observe(truth.Add(jitter))
			if k > 600 {
				if d := got.Sub(truth); d > worst {
					worst = d
				} else if -d > worst {
					worst = -d
				}
			}
		}
		if worst > 400*time.Microsecond {
			t.Errorf("%vppm: settled error up to %v", ppm, worst)
		}
	}
}

// An xrun or restart moves the DAC by far more than jitter: follow it at
// once rather than slewing for seconds.
func TestOutputClockFollowsAJump(t *testing.T) {
	period := 42 * time.Millisecond
	c := NewOutputClock(period)
	t0 := epoch.Add(time.Hour)
	for k := 0; k < 50; k++ {
		c.Observe(t0.Add(time.Duration(k) * period))
	}
	jumped := t0.Add(50*period + 100*time.Millisecond)
	if got := c.Observe(jumped); !got.Equal(jumped) {
		t.Errorf("after a 100ms jump, estimate is %v off", got.Sub(jumped))
	}
}

func TestOutDiagWindow(t *testing.T) {
	per := 42667 * time.Microsecond
	c := NewOutputClock(per)
	t0 := time.Unix(0, 0)
	for i := 0; i < 50; i++ {
		c.Observe(t0.Add(time.Duration(i) * per))
	}
	c.TakeDiag()                                     // converged; start a clean window
	c.Observe(t0.Add(50*per + 300*time.Microsecond)) // one measurement 300µs off
	c.Observe(t0.Add(51*per + 30*time.Millisecond))  // a jump: resets the loop
	d := c.TakeDiag()
	if d.N != 2 || d.BigResid != 2 || d.Resets != 1 || d.ResidMaxUs < 29000 || d.NudgeMaxUs == 0 {
		t.Fatalf("diag = %+v", d)
	}
	if again := c.TakeDiag(); again.N != 0 {
		t.Fatalf("window not reset: %+v", again)
	}
}
