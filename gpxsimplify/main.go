// Copyright 2016 Florian Pigorsch. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"flag"
	"fmt"
	"os"

	"github.com/ptrv/go-gpx"
)

func main() {
	flag.Usage = func() {
		fmt.Println("Usage: gpxsimplify [-dist DIST] INPUTFILE OUTPUTFILE")
		flag.PrintDefaults()
	}

	maxDist := flag.Float64("dist", 5, "Max distance (in meters) for track simplification. Must be >= 0.")
	flag.Parse()

	args := flag.Args()
	if len(args) != 2 {
		fmt.Println("Error: You must specify both INPUTFILE and OUTPUTFILE")
		flag.Usage()
		return
	} else if *maxDist < 0 {
		fmt.Println("Error: -dist must be >= 0")
		flag.Usage()
		return
	}

	gpxFile, err := gpx.ParseFile(args[0])
	if err != nil {
		fmt.Println("Error reading input GPX file:", err)
		return
	}

	gpxFile.SimplifyTracks(*maxDist)

	f, err := os.Create(args[1])
	if err != nil {
		fmt.Println("Error creating output GPX file:", err)
		return
	}
	defer f.Close()

	_, err = f.Write(gpxFile.ToXML())
	if err != nil {
		fmt.Println("Error writing output GPX file:", err)
		return
	}
}
