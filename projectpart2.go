package main

//Importing the different packages
import (
	"bufio"
	"fmt"
	"math"
	"math/rand"
	"strconv"
	"sync"
	"time"

	"os"
	//"sync"
)

// Createing the point3D structure with the x y and z variable
type Point3D struct {
	X float64
	Y float64
	Z float64
}

// Creating the plane 3D structure with the A, B,C and D variables
type Plane3D struct {
	A float64
	B float64
	C float64
	D float64
}

// Creating plane3DwSuppport that includes a plane3D and an integer value of the number of points it supports
type Plane3DwSupport struct {
	Plane3D
	SupportSize int
}

// reads an XYZ file and returns a Point3D list
func ReadXYZ(filename string) []Point3D {
	//Opening the file with the filename and checking for an error
	file, err := os.Open(filename)
	if err != nil {
		panic(err)
	}
	//Defering closing the file to end of function
	defer file.Close()

	//Initializing an array list of Point3Ds
	var points []Point3D

	//Creating a new scanner for the file
	scanner := bufio.NewScanner(file)

	//Scanning the first header line
	scanner.Scan()
	//For each line in the text file until the end
	for scanner.Scan() {
		//Creating the point for the line
		var point Point3D
		//Declaring the point variable to have the x,y, and z variables of the line in the text file
		_, err := fmt.Sscanf(scanner.Text(), "%f %f %f", &point.X, &point.Y, &point.Z)
		//Checking for error
		if err != nil {
			panic(err)
		}
		//Appenfing the point to the list of Point3Ds and then going to next line until end of file
		points = append(points, point)
	}
	//Returning the list of points
	return points

}

// makes a new XYZ file from Point3D list
func SaveXYZ(filename string, points []Point3D) {
	//Creating a new blank file
	file, err := os.Create(filename)

	//Checking for an error
	if err != nil {
		panic(err)
	}
	//Defering closing the file to end of function
	defer file.Close()

	//Declaring a new writer
	writer := bufio.NewWriter(file)

	//Writing the header line
	fmt.Fprintln(writer, "X           Y            Z")

	//For each point in the point list given
	for _, point := range points {
		//Writes a liine in the file with the three x, y and z variables of the points
		fmt.Fprintf(writer, "%f %f %f\n", float64(point.X), float64(point.Y), float64(point.Z))
	}
	writer.Flush()
}

// computes the distance between points p1 and p2
func (p1 *Point3D) GetDistance(p2 *Point3D) float64 {
	//Uses the distance between two points formula and returns the result using the two points
	return math.Sqrt(math.Pow(p1.X-p2.X, 2) + math.Pow(p1.Y-p2.Y, 2) + math.Pow(p1.Z-p2.Z, 2))
}

// computes the plane defined by a set of 3 points
func GetPlane(points [3]Point3D) Plane3D {
	//Calculating the vector of the point1-point0
	vector1 := Point3D{points[1].X - points[0].X, points[1].Y - points[0].Y, points[1].Z - points[0].Z}
	//Calculating the vector of point2-point0
	vector2 := Point3D{points[2].X - points[0].X, points[2].Y - points[0].Y, points[2].Z - points[0].Z}

	//Cross value of vector1 and vector2
	crossValue := Point3D{vector1.Y*vector2.Z - vector1.Z*vector2.Y, vector1.Z*vector2.X - vector1.X*vector2.Z, vector1.X*vector2.Y - vector1.Y*vector2.X}
	//Using the cross values and point 0 to isolate for d
	d := -1 * (crossValue.X*points[0].X + crossValue.Y*points[0].Y + crossValue.Z*points[0].Z)
	//Returning the plane3D
	return Plane3D{crossValue.X, crossValue.Y, crossValue.Z, d}

}

// computes the number of required RANSAC iterations
func GetNumberOfIterations(confidence float64,
	percentageOfPointsOnPlane float64) int {
	//Uses the numberOfIterations formula math.log(1-condifence)/math.log(1-percentage^3)

	//Calculating left side of divide
	leftSide := math.Log(1 - confidence)
	//Calculating right side of divide
	rightSide := math.Log(1 - math.Pow(percentageOfPointsOnPlane, 3))
	//Calculating left side divided by right side and rounding it up to nearest int
	k := int(math.Ceil(leftSide / rightSide))
	return k

}

// computes the support of a plane in a set of points
func GetSupport(plane Plane3D, points []Point3D,
	eps float64) Plane3DwSupport {
	//Initializing support size to 0
	sizeCount := 0
	//Looping through entire point list
	for i := 0; i < len(points); i++ {
		//Using distance from point to plane forula to find distance
		topValue := math.Abs((plane.A * points[i].X) + (plane.B * points[i].Y) + (plane.C*points[i].Z + plane.D))
		bottomValue := math.Sqrt((plane.A * plane.A) + (plane.B * plane.B) + (plane.C * plane.C))
		if topValue/bottomValue < eps { //If distance is less then eps add one to the support size count
			sizeCount++
		}
	}
	//Returning a Plane3DwSupport with the plane given in the parameter and the size count
	return Plane3DwSupport{plane, sizeCount}
}

// Function that returns the points in a plane as a list
func GetSupportingPoints(plane Plane3D, eps float64, points []Point3D) (pointList []Point3D) {
	//Initializing the list to an empty array
	pList := make([]Point3D, 0)

	//Loop through the given Point3D list
	for i := 0; i < len(points); i++ {
		//Finding the distance from point to plane
		topValue := math.Abs((plane.A * points[i].X) + (plane.B * points[i].Y) + (plane.C*points[i].Z + plane.D))
		bottomValue := math.Sqrt((plane.A * plane.A) + (plane.B * plane.B) + (plane.C * plane.C))
		if topValue/bottomValue < eps { //If the distance is less then eps add the point to the array

			pList = append(pList, points[i])

		}
	}
	//Return the array with the planes points
	return pList
}

// creates a new slice of points in which all points
// belonging to the plane have been removed
func RemovePlane(plane Plane3D, points []Point3D,
	eps float64) []Point3D {
	//Creating a new array for the points that are in the plane
	var removedFromArray []Point3D
	//Looping through Point3D list
	for i := 0; i < len(points); i++ {
		//Finding distance from plane to point
		topValue := math.Abs((plane.A * points[i].X) + (plane.B * points[i].Y) + (plane.C*points[i].Z + plane.D))
		bottomValue := math.Sqrt((plane.A * plane.A) + (plane.B * plane.B) + (plane.C * plane.C))
		//If distance is bigger then eps add it to the removed from array
		if topValue/bottomValue > eps {
			removedFromArray = append(removedFromArray, points[i])
		}
	}
	//Return the array that has all the points from before except those that are in the plane in paramater
	return removedFromArray
}

// Main function
func main() {
	//New point list from input file
	var points []Point3D = ReadXYZ(os.Args[1])
	//Read the confidence value and check for error
	confidence, err := strconv.ParseFloat(os.Args[2], 64)
	if err != nil {
		fmt.Println(err)
	}

	//Read the percentage value
	percentage, _ := strconv.ParseFloat(os.Args[3], 64)
	//Read the eps value
	eps, _ := strconv.ParseFloat(os.Args[4], 64)
	//Find number of iterations
	numberofIterations := GetNumberOfIterations(confidence, percentage)
	//Declaring best support plane as a Plane3DwSupport
	bestSupport := Plane3DwSupport{}

	//Declaring the various channels for the pipeline
	randomPoint := make(chan Point3D, len(points))
	tripletCh := make(chan [3]Point3D, len(points)/3)
	takeN := make(chan [3]Point3D, numberofIterations)
	planeChan := make(chan Plane3D, numberofIterations)
	supportChan0 := make(chan Plane3DwSupport, numberofIterations)
	supportChan1 := make(chan Plane3DwSupport, numberofIterations)
	supportChan2 := make(chan Plane3DwSupport, numberofIterations)
	supportChan3 := make(chan Plane3DwSupport, numberofIterations)
	supportChan4 := make(chan Plane3DwSupport, numberofIterations)
	supportChan5 := make(chan Plane3DwSupport, numberofIterations)
	supportChan6 := make(chan Plane3DwSupport, numberofIterations)
	supportChan7 := make(chan Plane3DwSupport, numberofIterations)
	supportChan8 := make(chan Plane3DwSupport, numberofIterations)
	supportChan9 := make(chan Plane3DwSupport, numberofIterations)
	supportChan10 := make(chan Plane3DwSupport, numberofIterations)
	supportChan11 := make(chan Plane3DwSupport, numberofIterations)
	supportChan12 := make(chan Plane3DwSupport, numberofIterations)
	supportChan13 := make(chan Plane3DwSupport, numberofIterations)

	done := make(chan struct{})

	//Creating a wait group
	wg := &sync.WaitGroup{}
	wg.Add(19)

	//Starting the pipeline
	start := time.Now()

	//First go function, random number generator
	go func() {
		defer wg.Done()
		defer close(randomPoint)
		for range points { //For points in the points list
			select { //Either finish or go to default
			case <-done:

				return
			default:
				//Add a random point from the point list to the randomPoint channel
				randomPoint <- points[rand.Intn(len(points))]
			}
		}

	}()

	//Go func that uses the random point channel to create random triplets
	go func() {
		defer wg.Done()
		defer close(tripletCh)
		//Triplet created as a size 3 array
		triplet := [3]Point3D{}
		i := 0
		for range randomPoint { //For the randomPoint channel values
			select {
			case <-done:

				return
			default:
				//Randompoint becomes one of the triplet values
				triplet[i] = <-randomPoint
				//The triplet restarts every 3rd loop
				i = (i + 1) % 3
				//After the last point in the triplett add the triplet to the triplet channel
				if i == 2 {
					tripletCh <- triplet

				}
			}
		}

	}()
	//Makes sure the pipeline stops after n iterations
	go func() {
		defer wg.Done()
		defer close(takeN)
		//Loops through number of iterations times
		for i := 0; i < numberofIterations; i++ {
			//Changes the channel of the triplet
			triplet := <-tripletCh
			takeN <- triplet

		}
		//Ends and also stops the other go functions making it so it doesn't deadlock

	}()

	//Go function that estimates the plane for the triplet
	go func() {
		defer wg.Done()
		defer close(planeChan)
		//For triplets in the takeN channel
		for triplet := range takeN {
			select {
			case <-done:

				return
			default:
				//Use the getPlane function from the triplet to get the plane
				plane := GetPlane(triplet)
				//Add the plane to the plane channel
				planeChan <- plane

			}
		}
	}()

	//Go function to find the supporting points

	go func() {

		defer wg.Done()
		defer close(supportChan0)

		//For planes in the plane channel
		for plane := range planeChan {
			select {
			case <-done:
				return
			default:
				//Creates a Plane3DwSupport variable
				var planewSupport Plane3DwSupport

				//Intializes the plane with support using the GetSupport method and the plane from the plane channel
				planewSupport = GetSupport(plane, points, eps)

				//Adding the plane with support to the support channel
				supportChan0 <- planewSupport
			}
		}

	}()

	go func() {

		defer wg.Done()
		defer close(supportChan1)

		//For planes in the plane channel
		for plane := range planeChan {
			select {
			case <-done:
				return
			default:
				//Creates a Plane3DwSupport variable
				var planewSupport Plane3DwSupport

				//Intializes the plane with support using the GetSupport method and the plane from the plane channel
				planewSupport = GetSupport(plane, points, eps)

				//Adding the plane with support to the support channel
				supportChan1 <- planewSupport
			}
		}

	}()
	go func() {

		defer wg.Done()
		defer close(supportChan2)

		//For planes in the plane channel
		for plane := range planeChan {
			select {
			case <-done:
				return
			default:
				//Creates a Plane3DwSupport variable
				var planewSupport Plane3DwSupport

				//Intializes the plane with support using the GetSupport method and the plane from the plane channel
				planewSupport = GetSupport(plane, points, eps)

				//Adding the plane with support to the support channel
				supportChan2 <- planewSupport
			}
		}

	}()

	go func() {

		defer wg.Done()
		defer close(supportChan3)

		//For planes in the plane channel
		for plane := range planeChan {
			select {
			case <-done:
				return
			default:
				//Creates a Plane3DwSupport variable
				var planewSupport Plane3DwSupport

				//Intializes the plane with support using the GetSupport method and the plane from the plane channel
				planewSupport = GetSupport(plane, points, eps)

				//Adding the plane with support to the support channel
				supportChan3 <- planewSupport
			}
		}

	}()
	go func() {

		defer wg.Done()
		defer close(supportChan4)

		//For planes in the plane channel
		for plane := range planeChan {
			select {
			case <-done:
				return
			default:
				//Creates a Plane3DwSupport variable
				var planewSupport Plane3DwSupport

				//Intializes the plane with support using the GetSupport method and the plane from the plane channel
				planewSupport = GetSupport(plane, points, eps)

				//Adding the plane with support to the support channel
				supportChan4 <- planewSupport
			}
		}

	}()
	go func() {

		defer wg.Done()
		defer close(supportChan5)

		//For planes in the plane channel
		for plane := range planeChan {
			select {
			case <-done:
				return
			default:
				//Creates a Plane3DwSupport variable
				var planewSupport Plane3DwSupport

				//Intializes the plane with support using the GetSupport method and the plane from the plane channel
				planewSupport = GetSupport(plane, points, eps)

				//Adding the plane with support to the support channel
				supportChan5 <- planewSupport
			}
		}

	}()

	go func() {

		defer wg.Done()
		defer close(supportChan6)

		//For planes in the plane channel
		for plane := range planeChan {
			select {
			case <-done:
				return
			default:
				//Creates a Plane3DwSupport variable
				var planewSupport Plane3DwSupport

				//Intializes the plane with support using the GetSupport method and the plane from the plane channel
				planewSupport = GetSupport(plane, points, eps)

				//Adding the plane with support to the support channel
				supportChan6 <- planewSupport
			}
		}

	}()

	go func() {

		defer wg.Done()
		defer close(supportChan7)

		//For planes in the plane channel
		for plane := range planeChan {
			select {
			case <-done:
				return
			default:
				//Creates a Plane3DwSupport variable
				var planewSupport Plane3DwSupport

				//Intializes the plane with support using the GetSupport method and the plane from the plane channel
				planewSupport = GetSupport(plane, points, eps)

				//Adding the plane with support to the support channel
				supportChan7 <- planewSupport
			}
		}

	}()

	go func() {

		defer wg.Done()
		defer close(supportChan8)

		//For planes in the plane channel
		for plane := range planeChan {
			select {
			case <-done:
				return
			default:
				//Creates a Plane3DwSupport variable
				var planewSupport Plane3DwSupport

				//Intializes the plane with support using the GetSupport method and the plane from the plane channel
				planewSupport = GetSupport(plane, points, eps)

				//Adding the plane with support to the support channel
				supportChan8 <- planewSupport
			}
		}

	}()

	go func() {

		defer wg.Done()
		defer close(supportChan9)

		//For planes in the plane channel
		for plane := range planeChan {
			select {
			case <-done:
				return
			default:
				//Creates a Plane3DwSupport variable
				var planewSupport Plane3DwSupport

				//Intializes the plane with support using the GetSupport method and the plane from the plane channel
				planewSupport = GetSupport(plane, points, eps)

				//Adding the plane with support to the support channel
				supportChan9 <- planewSupport
			}
		}

	}()

	go func() {

		defer wg.Done()
		defer close(supportChan11)

		//For planes in the plane channel
		for plane := range planeChan {
			select {
			case <-done:
				return
			default:
				//Creates a Plane3DwSupport variable
				var planewSupport Plane3DwSupport

				//Intializes the plane with support using the GetSupport method and the plane from the plane channel
				planewSupport = GetSupport(plane, points, eps)

				//Adding the plane with support to the support channel
				supportChan11 <- planewSupport
			}
		}

	}()

	go func() {

		defer wg.Done()
		defer close(supportChan10)

		//For planes in the plane channel
		for plane := range planeChan {
			select {
			case <-done:
				return
			default:
				//Creates a Plane3DwSupport variable
				var planewSupport Plane3DwSupport

				//Intializes the plane with support using the GetSupport method and the plane from the plane channel
				planewSupport = GetSupport(plane, points, eps)

				//Adding the plane with support to the support channel
				supportChan10 <- planewSupport
			}
		}

	}()

	go func() {

		defer wg.Done()
		defer close(supportChan12)

		//For planes in the plane channel
		for plane := range planeChan {
			select {
			case <-done:
				return
			default:
				//Creates a Plane3DwSupport variable
				var planewSupport Plane3DwSupport

				//Intializes the plane with support using the GetSupport method and the plane from the plane channel
				planewSupport = GetSupport(plane, points, eps)

				//Adding the plane with support to the support channel
				supportChan12 <- planewSupport
			}
		}

	}()

	go func() {

		defer wg.Done()
		defer close(supportChan13)

		//For planes in the plane channel
		for plane := range planeChan {
			select {
			case <-done:
				return
			default:
				//Creates a Plane3DwSupport variable
				var planewSupport Plane3DwSupport

				//Intializes the plane with support using the GetSupport method and the plane from the plane channel
				planewSupport = GetSupport(plane, points, eps)

				//Adding the plane with support to the support channel
				supportChan13 <- planewSupport
			}
		}

	}()

	//Go function to determine dominant plane
	go func() {
		defer wg.Done()
		//Checks the planes coming in from the support channel
		for plane := range fanIn(supportChan0, supportChan1, supportChan2, supportChan3, supportChan4, supportChan5, supportChan6, supportChan7, supportChan8, supportChan9, supportChan10, supportChan11, supportChan12, supportChan13) {
			select {
			case <-done:
				return
			default:

				//If the support value from the plane is bigger then the biggest support value, it becomes the new bestSupport plane
				if plane.SupportSize > bestSupport.SupportSize {
					bestSupport = plane

				}

			}
		}
	}()

	//Wait for everything to finish
	wg.Wait()
	elapsed := time.Since(start)
	fmt.Println(elapsed)
	//Creating a list for all the points in the bestSupport plane
	var bestSupportList []Point3D

	fmt.Println(bestSupport.SupportSize)

	//Initializing the bestSupportlist using the GetSupportingPoints method frmom before
	bestSupportList = GetSupportingPoints(bestSupport.Plane3D, eps, points)

	//Saving the dominant plane into an XYZ file
	SaveXYZ(os.Args[1]+"_p", bestSupportList)

	//Saving points that aren't in the dominant plane into an XYZ file using the RemovePlane method
	SaveXYZ(os.Args[1]+"_p0", RemovePlane(bestSupport.Plane3D, points, eps))

}

// Function that combines and merges multiple input channels
func fanIn(supportChannels ...<-chan Plane3DwSupport) <-chan Plane3DwSupport {
	//Declaring a wait group
	var wg sync.WaitGroup
	//Declaring the output channel
	outChannel := make(chan Plane3DwSupport)

	//Function to add the channels to the out channel
	output := func(channel <-chan Plane3DwSupport) {
		for n := range channel {
			outChannel <- n
		}
		wg.Done()
	}
	//Adding the time needed to the wait group
	wg.Add(len(supportChannels))
	//Adding the support channel to the main output channel
	for _, i := range supportChannels {
		go output(i)
	}
	//Waiting to sync
	go func() {
		wg.Wait()
		close(outChannel)
	}()
	//Returning the main output
	return outChannel
}
