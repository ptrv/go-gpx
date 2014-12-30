// Copyright 2013, 2014 Peter Vasil. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Package gpx implements a simple GPX parser.
package gpx

import (
	"bytes"
	"encoding/xml"
	"fmt"
	"io"
	"math"
	"os"
	"time"
)

/*==========================================================*/

const timelayout = "2006-01-02T15:04:05Z"
const defaultStoppedSpeedThreshold = 1.0

/*==========================================================*/

// Waypoints is a collection of waypoints whether in a track, a route, or standalone.
type Waypoints []Wpt

// Trkseg is a GPX track segment
type Trkseg struct {
	XMLName   xml.Name `xml:"trkseg"`
	Waypoints `xml:"trkpt"`
}

// Trk is a GPX track
type Trk struct {
	XMLName  xml.Name `xml:"trk"`
	Name     string   `xml:"name,omitempty"`
	Cmt      string   `xml:"cmt,omitempty"`
	Desc     string   `xml:"desc,omitempty"`
	Src      string   `xml:"src,omitempty"`
	Links    []Link   `xml:"link"`
	Number   int      `xml:"number,omitempty"`
	Type     string   `xml:"type,omitempty"`
	Segments []Trkseg `xml:"trkseg"`
}

// Wpt is a GPX waypoint
type Wpt struct {
	Lat float64 `xml:"lat,attr"`
	Lon float64 `xml:"lon,attr"`
	// Position info
	Ele         float64 `xml:"ele,omitempty"`
	Timestamp   string  `xml:"time,omitempty"`
	MagVar      string  `xml:"magvar,omitempty"`
	GeoIDHeight string  `xml:"geoidheight,omitempty"`
	// Description info
	Name  string `xml:"name,omitempty"`
	Cmt   string `xml:"cmt,omitempty"`
	Desc  string `xml:"desc,omitempty"`
	Src   string `xml:"src,omitempty"`
	Links []Link `xml:"link"`
	Sym   string `xml:"sym,omitempty"`
	Type  string `xml:"type,omitempty"`
	// Accuracy info
	Fix          string  `xml:"fix,omitempty"`
	Sat          int     `xml:"sat,omitempty"`
	Hdop         float64 `xml:"hdop,omitempty"`
	Vdop         float64 `xml:"vdop,omitempty"`
	Pdop         float64 `xml:"pdop,omitempty"`
	AgeOfGpsData float64 `xml:"ageofgpsdata,omitempty"`
	DGpsID       int     `xml:"dgpsid,omitempty"`
}

// Rte is a GPX Route
type Rte struct {
	XMLName   xml.Name `xml:"rte"`
	Name      string   `xml:"name,omitempty"`
	Cmt       string   `xml:"cmt,omitempty"`
	Desc      string   `xml:"desc,omitempty"`
	Src       string   `xml:"src,omitempty"`
	Links     []Link   `xml:"link"`
	Number    int      `xml:"number,omitempty"`
	Type      string   `xml:"type,omitempty"`
	Waypoints `xml:"rtept"`
}

// Link is a GPX link
type Link struct {
	XMLName xml.Name `xml:"link"`
	URL     string   `xml:"href,attr,omitempty"`
	Text    string   `xml:"text,omitempty"`
	Type    string   `xml:"type,omitempty"`
}

// Copyright is a GPX copyright tag
type Copyright struct {
	XMLName xml.Name `xml:"copyright"`
	Author  string   `xml:"author,attr"`
	Year    string   `xml:"year,omitempty"`
	License string   `xml:"license,omitempty"`
}

// Email is a GPX email tag
type Email struct {
	XMLName xml.Name `xml:"email"`
	ID      string   `xml:"id,attr,omitempty"`
	Domain  string   `xml:"domain,attr,omitempty"`
}

// Person is a GPX person tag
type Person struct {
	XMLName xml.Name `xml:"author"`
	Name    string   `xml:"name,omitempty"`
	Email   *Email   `xml:"email,omitempty"`
	Link    *Link    `xml:"link,omitempty"`
}

// Metadata is a GPX metadata tag
type Metadata struct {
	XMLName   xml.Name   `xml:"metadata"`
	Name      string     `xml:"name,omitempty"`
	Desc      string     `xml:"desc,omitempty"`
	Author    *Person    `xml:"author,omitempty"`
	Copyright *Copyright `xml:"copyright,omitempty"`
	Links     []Link     `xml:"link,omitempty"`
	Timestamp string     `xml:"time,omitempty"`
	Keywords  string     `xml:"keywords,omitempty"`
	Bounds    *Bounds    `xml:"bounds"`
}

// Gpx represents the root of a GPX file
type Gpx struct {
	XMLName      xml.Name  `xml:"http://www.topografix.com/GPX/1/1 gpx"`
	XMLNsXsi     string    `xml:"xmlns:xsi,attr,omitempty"`
	XMLSchemaLoc string    `xml:"xsi:schemaLocation,attr,omitempty"`
	Version      string    `xml:"version,attr"`
	Creator      string    `xml:"creator,attr"`
	Metadata     *Metadata `xml:"metadata,omitempty"`
	Waypoints    Waypoints `xml:"wpt,omitempty"`
	Routes       []Rte     `xml:"rte,omitempty"`
	Tracks       []Trk     `xml:"trk"`
}

// Bounds is a GPX bounds tag
type Bounds struct {
	XMLName xml.Name `xml:"bounds"`
	MinLat  float64  `xml:"minlat,attr"`
	MaxLat  float64  `xml:"maxlat,attr"`
	MinLon  float64  `xml:"minlon,attr"`
	MaxLon  float64  `xml:"maxlon,attr"`
}

/*==========================================================*/

// TimeBounds represents a start/end time range
type TimeBounds struct {
	StartTime time.Time
	EndTime   time.Time
}

// UphillDownhill represents an uphill/downhill range
type UphillDownhill struct {
	Uphill   float64
	Downhill float64
}

// MovingData represents moving data
type MovingData struct {
	MovingTime      float64
	StoppedTime     float64
	MovingDistance  float64
	StoppedDistance float64
	MaxSpeed        float64
}

// SpeedsAndDistances represents speed and distance
type SpeedsAndDistances struct {
	Speed    float64
	Distance float64
}

// LocationsResultPair represents a result from location query
type LocationsResultPair struct {
	SegmentNo int
	PointNo   int
}

/*==========================================================*/

// Parse parses a GPX reader and return a Gpx object.
func Parse(r io.Reader) (*Gpx, error) {
	g := NewGpx()
	d := xml.NewDecoder(r)
	err := d.Decode(g)
	if err != nil {
		return nil, fmt.Errorf("couldn't parse gpx data: %v", err)
	}
	return g, nil
}

// ParseFile reads a GPX file and parses it.
func ParseFile(filepath string) (*Gpx, error) {
	f, err := os.Open(filepath)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	return Parse(f)
}

/*==========================================================*/

func getTime(timestr string) time.Time {
	t, err := time.Parse(timelayout, timestr)
	if err != nil {
		return time.Time{}
	}
	return t
}

func toXML(n interface{}) []byte {
	content, _ := xml.MarshalIndent(n, "", "	")
	return content
}

func maxBounds() *Bounds {
	return &Bounds{
		MaxLat: -math.MaxFloat64,
		MinLat: math.MaxFloat64,
		MaxLon: -math.MaxFloat64,
		MinLon: math.MaxFloat64,
	}
}

/*==========================================================*/

func (tb *TimeBounds) String() string {
	return fmt.Sprintf("%+v, %+v", tb.StartTime, tb.EndTime)
}

func (b *Bounds) merge(b2 *Bounds) {
	b.MaxLat = math.Max(b.MaxLat, b2.MaxLat)
	b.MinLat = math.Min(b.MinLat, b2.MinLat)
	b.MaxLon = math.Max(b.MaxLon, b2.MaxLon)
	b.MinLon = math.Min(b.MinLon, b2.MinLon)
}

func (b *Bounds) String() string {
	return fmt.Sprintf("Max: %+v, %+v Min: %+v, %+v",
		b.MinLat, b.MinLon, b.MaxLat, b.MaxLon)
}

/*==========================================================*/

// NewGpx creates and returns a new Gpx objects.
func NewGpx() *Gpx {
	gpx := new(Gpx)
	gpx.XMLNsXsi = "http://www.w3.org/2001/XMLSchema-instance"
	gpx.XMLSchemaLoc = "http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd"
	gpx.Version = "1.1"
	gpx.Creator = "https://github.com/ptrv/go-gpx"
	return gpx
}

// Clone duplicates a Gpx object with deep copy.
func (g *Gpx) Clone() *Gpx {
	newgpx := NewGpx()
	if g.Metadata != nil {
		newgpx.Metadata = &Metadata{
			Name:      g.Metadata.Name,
			Desc:      g.Metadata.Desc,
			Links:     make([]Link, len(g.Metadata.Links)),
			Timestamp: g.Metadata.Timestamp,
			Keywords:  g.Metadata.Keywords,
		}
		copy(newgpx.Metadata.Links, g.Metadata.Links)
		if g.Metadata.Author != nil {
			newgpx.Metadata.Author = &Person{
				Name: g.Metadata.Author.Name,
			}
			if g.Metadata.Author.Email != nil {
				newgpx.Metadata.Author.Email = &Email{
					ID:     g.Metadata.Author.Email.ID,
					Domain: g.Metadata.Author.Email.Domain,
				}
			}
			if g.Metadata.Author.Link != nil {
				newgpx.Metadata.Author.Link = &Link{
					URL:  g.Metadata.Author.Link.URL,
					Text: g.Metadata.Author.Link.Text,
					Type: g.Metadata.Author.Link.Type,
				}
			}
		}
		if g.Metadata.Copyright != nil {
			newgpx.Metadata.Copyright = &Copyright{
				Author:  g.Metadata.Copyright.Author,
				Year:    g.Metadata.Copyright.Year,
				License: g.Metadata.Copyright.License,
			}
		}
		if g.Metadata.Bounds != nil {
			newgpx.Metadata.Bounds = &Bounds{
				MaxLat: g.Metadata.Bounds.MaxLat,
				MinLat: g.Metadata.Bounds.MinLat,
				MaxLon: g.Metadata.Bounds.MaxLon,
				MinLon: g.Metadata.Bounds.MinLon,
			}
		}
	}

	newgpx.Waypoints = make([]Wpt, len(g.Waypoints))
	newgpx.Routes = make([]Rte, len(g.Routes))
	newgpx.Tracks = make([]Trk, len(g.Tracks))
	copy(newgpx.Waypoints, g.Waypoints)
	copy(newgpx.Routes, g.Routes)
	copy(newgpx.Tracks, g.Tracks)

	return newgpx
}

// Length2D returns the 2D length of all tracks in a Gpx.
func (g *Gpx) Length2D() float64 {
	var length2d float64
	for _, trk := range g.Tracks {
		length2d += trk.Length2D()
	}
	return length2d
}

// Length3D returns the 3D length of all tracks,
func (g *Gpx) Length3D() float64 {
	var length3d float64
	for _, trk := range g.Tracks {
		length3d += trk.Length3D()
	}
	return length3d
}

// TimeBounds returns the time bounds of all tacks in a Gpx.
func (g *Gpx) TimeBounds() *TimeBounds {
	var tbGpx *TimeBounds
	for i, trk := range g.Tracks {
		tbTrk := trk.TimeBounds()
		if i == 0 {
			tbGpx = trk.TimeBounds()
		} else {
			tbGpx.EndTime = tbTrk.EndTime
		}
	}
	return tbGpx
}

// Bounds returns the bounds of all tracks in a Gpx.
func (g *Gpx) Bounds() *Bounds {
	b := maxBounds()
	for _, trk := range g.Tracks {
		b.merge(trk.Bounds())
	}
	return b
}

// MovingData returns the moving data for all tracks in a Gpx.
func (g *Gpx) MovingData() *MovingData {
	var (
		movingTime      float64
		stoppedTime     float64
		movingDistance  float64
		stoppedDistance float64
		maxSpeed        float64
	)

	for _, trk := range g.Tracks {
		md := trk.MovingData()
		movingTime += md.MovingTime
		stoppedTime += md.StoppedTime
		movingDistance += md.MovingDistance
		stoppedDistance += md.StoppedDistance

		if md.MaxSpeed > maxSpeed {
			maxSpeed = md.MaxSpeed
		}
	}
	return &MovingData{
		MovingTime:      movingTime,
		MovingDistance:  movingDistance,
		StoppedTime:     stoppedTime,
		StoppedDistance: stoppedDistance,
		MaxSpeed:        maxSpeed,
	}

}

// Split splits the Gpx segment segNo in a given track trackNo at
// pointNo.
func (g *Gpx) Split(trackNo, segNo, pointNo int) {
	if trackNo >= len(g.Tracks) {
		return
	}

	track := &g.Tracks[trackNo]

	track.Split(segNo, pointNo)
}

// Duration returns the duration of all tracks in a Gpx in seconds.
func (g *Gpx) Duration() float64 {
	if len(g.Tracks) == 0 {
		return 0.0
	}
	var result float64
	for _, trk := range g.Tracks {
		result += trk.Duration()
	}

	return result
}

// UphillDownhill returns uphill and downhill values for all tracks in a
// Gpx.
func (g *Gpx) UphillDownhill() *UphillDownhill {
	if len(g.Tracks) == 0 {
		return nil
	}

	var (
		uphill   float64
		downhill float64
	)

	for _, trk := range g.Tracks {
		updo := trk.UphillDownhill()

		uphill += updo.Uphill
		downhill += updo.Downhill
	}

	return &UphillDownhill{
		Uphill:   uphill,
		Downhill: downhill,
	}
}

// LocationAt returns a LocationResultsPair consisting the segment index
// and the GpxWpt at a certain time.
func (g *Gpx) LocationAt(t time.Time) []LocationsResultPair {
	var results []LocationsResultPair

	for _, trk := range g.Tracks {
		locs := trk.LocationAt(t)
		if len(locs) > 0 {
			results = append(results, locs...)
		}
	}
	return results
}

// ToXML returns the marshalled Gpx object.
func (g *Gpx) ToXML() []byte {
	var buffer bytes.Buffer
	buffer.WriteString(xml.Header)
	buffer.Write(toXML(g))
	return buffer.Bytes()
}

/*==========================================================*/

// Length2D returns the 2D length of a GPX track.
func (trk *Trk) Length2D() float64 {
	var l float64
	for _, seg := range trk.Segments {
		l += seg.Length2D()
	}
	return l
}

// Length3D returns the 3D length of a GPX track.
func (trk *Trk) Length3D() float64 {
	var l float64
	for _, seg := range trk.Segments {
		l += seg.Length3D()
	}
	return l
}

// TimeBounds returns the time bounds of a GPX track.
func (trk *Trk) TimeBounds() *TimeBounds {
	var tbTrk *TimeBounds

	for i, seg := range trk.Segments {
		tbSeg := seg.TimeBounds()
		if i == 0 {
			tbTrk = tbSeg
		} else {
			tbTrk.EndTime = tbSeg.EndTime
		}
	}
	return tbTrk
}

// Bounds returns the bounds of a GPX track.
func (trk *Trk) Bounds() *Bounds {
	b := maxBounds()
	for _, seg := range trk.Segments {
		b.merge(seg.Bounds())
	}
	return b
}

// Split splits a GPX segment at a point number ptNo in a GPX track.
func (trk *Trk) Split(segNo, ptNo int) {
	lenSegs := len(trk.Segments)
	if segNo >= lenSegs {
		return
	}

	var newSegs []Trkseg
	for i := 0; i < lenSegs; i++ {
		seg := trk.Segments[i]

		if i == segNo && ptNo < len(seg.Waypoints) {
			seg1, seg2 := seg.Split(ptNo)
			newSegs = append(newSegs, *seg1, *seg2)
		} else {
			newSegs = append(newSegs, seg)
		}
	}
	trk.Segments = newSegs
}

// Join joins two GPX segments in a GPX track.
func (trk *Trk) Join(segNo, segNo2 int) {
	lenSegs := len(trk.Segments)
	if segNo >= lenSegs && segNo2 >= lenSegs {
		return
	}
	var newSegs []Trkseg
	for i := 0; i < lenSegs; i++ {
		seg := trk.Segments[i]
		if i == segNo {
			secondSeg := trk.Segments[segNo2]
			seg.Join(&secondSeg)
			newSegs = append(newSegs, seg)
		} else if i == segNo2 {
			// do nothing, its already joined
		} else {
			newSegs = append(newSegs, seg)
		}
	}
	trk.Segments = newSegs
}

// JoinNext joins a GPX segment with the next segment in the current GPX
// track.
func (trk *Trk) JoinNext(segNo int) {
	trk.Join(segNo, segNo+1)
}

// MovingData returns the moving data of a GPX track.
func (trk *Trk) MovingData() *MovingData {
	var (
		movingTime      float64
		stoppedTime     float64
		movingDistance  float64
		stoppedDistance float64
		maxSpeed        float64
	)

	for _, seg := range trk.Segments {
		md := seg.MovingData()
		movingTime += md.MovingTime
		stoppedTime += md.StoppedTime
		movingDistance += md.MovingDistance
		stoppedDistance += md.StoppedDistance

		if md.MaxSpeed > maxSpeed {
			maxSpeed = md.MaxSpeed
		}
	}
	return &MovingData{
		MovingTime:      movingTime,
		MovingDistance:  movingDistance,
		StoppedTime:     stoppedTime,
		StoppedDistance: stoppedDistance,
		MaxSpeed:        maxSpeed,
	}
}

// Duration returns the duration of a GPX track.
func (trk *Trk) Duration() float64 {
	var result float64
	for _, seg := range trk.Segments {
		result += seg.Duration()
	}
	return result
}

// UphillDownhill return the uphill and downhill values of a GPX track.
func (trk *Trk) UphillDownhill() *UphillDownhill {
	var (
		uphill   float64
		downhill float64
	)

	for _, seg := range trk.Segments {
		updo := seg.UphillDownhill()

		uphill += updo.Uphill
		downhill += updo.Downhill
	}

	return &UphillDownhill{
		Uphill:   uphill,
		Downhill: downhill,
	}
}

// LocationAt returns a LocationResultsPair for a given time.
func (trk *Trk) LocationAt(t time.Time) []LocationsResultPair {
	var results []LocationsResultPair

	for i := 0; i < len(trk.Segments); i++ {
		seg := trk.Segments[i]
		loc := seg.LocationAt(t)
		if loc != -1 {
			results = append(results, LocationsResultPair{i, loc})
		}
	}
	return results
}

/*==========================================================*/

// Length2D returns the 2D length of a GPX segment.
func (w Waypoints) Length2D() float64 {
	var res float64
	for i := 1; i < len(w); i++ {
		res += w[i].Distance2D(&w[i-1])
	}
	return res
}

// Length3D returns the 3D length of a GPX segment.
func (w Waypoints) Length3D() float64 {
	var res float64
	for i := 1; i < len(w); i++ {
		res += w[i].Distance3D(&w[i-1])
	}
	return res
}

// TimeBounds returns the time bounds of a GPX segment.
func (w Waypoints) TimeBounds() *TimeBounds {
	var timeTuple []time.Time

	for _, trkpt := range w {
		if trkpt.Timestamp != "" {
			if len(timeTuple) < 2 {
				timeTuple = append(timeTuple, trkpt.Time())
			} else {
				timeTuple[1] = trkpt.Time()
			}
		}
	}
	if len(timeTuple) == 2 {
		return &TimeBounds{StartTime: timeTuple[0], EndTime: timeTuple[1]}
	}
	return nil
}

// Bounds returns the bounds of a GPX segment.
func (w Waypoints) Bounds() *Bounds {
	b := maxBounds()
	for _, pt := range w {
		b.merge(&Bounds{
			MaxLat: pt.Lat, MinLat: pt.Lat,
			MaxLon: pt.Lon, MinLon: pt.Lon,
		})
	}
	return b
}

// Speed returns the speed at point number in a GPX segment.
func (w Waypoints) Speed(pointIdx int) float64 {
	trkptsLen := len(w)
	if pointIdx >= trkptsLen {
		pointIdx = trkptsLen - 1
	}

	point := w[pointIdx]

	var prevPt *Wpt
	var nextPt *Wpt

	havePrev := false
	haveNext := false
	if 0 < pointIdx && pointIdx < trkptsLen {
		prevPt = &w[pointIdx-1]
		havePrev = true
	}

	if 0 < pointIdx && pointIdx < trkptsLen-1 {
		nextPt = &w[pointIdx+1]
		haveNext = true
	}

	haveSpeed1 := false
	haveSpeed2 := false

	var speed1 float64
	var speed2 float64
	if havePrev {
		speed1 = math.Abs(point.SpeedBetween(prevPt, true))
		haveSpeed1 = true
	}
	if haveNext {
		speed2 = math.Abs(point.SpeedBetween(nextPt, true))
		haveSpeed2 = true
	}

	if haveSpeed1 && haveSpeed2 {
		return (speed1 + speed2) / 2.0
	}

	if haveSpeed1 {
		return speed1
	}
	return speed2
}

// Duration returns the duration in seconds in a GPX segment.
func (w Waypoints) Duration() float64 {
	if len(w) == 0 {
		return 0.0
	}

	first := w[0]
	last := w[len(w)-1]

	if last.Time().Equal(first.Time()) ||
		last.Time().Before(first.Time()) {
		return 0.0
	}
	dur := last.Time().Sub(first.Time())
	return dur.Seconds()
}

// Elevations returns a slice with the elevations in a GPX segment.
func (w Waypoints) Elevations() []float64 {
	elevations := make([]float64, len(w))
	for i := range w {
		elevations[i] = w[i].Ele
	}
	return elevations
}

// UphillDownhill returns uphill and dowhill in a GPX segment.
func (w Waypoints) UphillDownhill() *UphillDownhill {
	if len(w) == 0 {
		return nil
	}

	uphill, downhill := calcUphillDownhill(w.Elevations())

	return &UphillDownhill{Uphill: uphill, Downhill: downhill}
}

// LocationAt returns the GpxWpt at a given time.
func (w Waypoints) LocationAt(t time.Time) int {
	lenPts := len(w)
	if lenPts == 0 {
		return -1
	}
	firstT := w[0]
	lastT := w[lenPts-1]
	if firstT.Time().Equal(lastT.Time()) || firstT.Time().After(lastT.Time()) {
		return -1
	}

	for i := 0; i < len(w); i++ {
		pt := w[i]
		if t.Before(pt.Time()) {
			return i
		}
	}

	return -1
}

// MovingData returns the moving data of a GPX segment.
func (w Waypoints) MovingData() *MovingData {
	var (
		movingTime      float64
		stoppedTime     float64
		movingDistance  float64
		stoppedDistance float64
	)

	var speedsDistances []SpeedsAndDistances

	for i := 1; i < len(w); i++ {
		prev := &w[i-1]
		pt := &w[i]

		dist := pt.Distance3D(prev)

		timedelta := pt.Time().Sub(prev.Time())
		seconds := timedelta.Seconds()
		var speedKmh float64

		if seconds > 0 {
			speedKmh = (dist / 1000.0) / (timedelta.Seconds() / math.Pow(60, 2))
		}

		if speedKmh <= defaultStoppedSpeedThreshold {
			stoppedTime += timedelta.Seconds()
			stoppedDistance += dist
		} else {
			movingTime += timedelta.Seconds()
			movingDistance += dist

			sd := SpeedsAndDistances{dist / timedelta.Seconds(), dist}
			speedsDistances = append(speedsDistances, sd)
		}
	}

	var maxSpeed float64
	if len(speedsDistances) > 0 {
		maxSpeed = calcMaxSpeed(speedsDistances)
	}

	return &MovingData{
		movingTime,
		stoppedTime,
		movingDistance,
		stoppedDistance,
		maxSpeed,
	}
}

// Center returns the center of a GPX route.
func (w Waypoints) Center() (lat, lon float64) {
	if len(w) == 0 {
		return 0, 0
	}

	for _, pt := range w {
		lat += pt.Lat
		lon += pt.Lon
	}

	n := float64(len(w))
	return lat / n, lon / n
}

/*==========================================================*/

// Time returns a timestamp string as Time object.
func (pt *Wpt) Time() time.Time {
	return getTime(pt.Timestamp)
}

// TimeDiff returns the time difference of two GpxWpts in seconds.
func (pt *Wpt) TimeDiff(pt2 *Wpt) float64 {
	t1 := pt.Time()
	t2 := pt2.Time()

	if t1.Equal(t2) {
		return 0.0
	}

	var delta time.Duration
	if t1.After(t2) {
		delta = t1.Sub(t2)
	} else {
		delta = t2.Sub(t1)
	}

	return delta.Seconds()
}

// SpeedBetween calculates the speed between two GpxWpts.
func (pt *Wpt) SpeedBetween(pt2 *Wpt, threeD bool) float64 {
	seconds := pt.TimeDiff(pt2)
	var distLen float64
	if threeD {
		distLen = pt.Distance3D(pt2)
	} else {
		distLen = pt.Distance2D(pt2)
	}

	return distLen / seconds
}

// Distance2D returns the 2D distance of two GpxWpts.
func (pt *Wpt) Distance2D(pt2 *Wpt) float64 {
	return distance(pt.Lat, pt.Lon, 0.0, pt2.Lat, pt2.Lon, 0.0, false, false)
}

// Distance3D returns the 3D distance of two GpxWpts.
func (pt *Wpt) Distance3D(pt2 *Wpt) float64 {
	return distance(pt.Lat, pt.Lon, pt.Ele, pt2.Lat, pt2.Lon, pt2.Ele, true, false)
}

// MaxDilutionOfPrecision returns the dilution precision of a GpxWpt.
func (pt *Wpt) MaxDilutionOfPrecision() float64 {
	return math.Max(pt.Hdop, math.Max(pt.Vdop, pt.Pdop))
}

/*==========================================================*/

// Split splits a GPX segment at point index i. Point i remains in
// first part.
func (seg *Trkseg) Split(i int) (*Trkseg, *Trkseg) {
	return &Trkseg{Waypoints: seg.Waypoints[:i+1]}, &Trkseg{Waypoints: seg.Waypoints[i+1:]}
}

// Join concatenates to GPX segments.
func (seg *Trkseg) Join(seg2 *Trkseg) {
	seg.Waypoints = append(seg.Waypoints, seg2.Waypoints...)
}
