// System Headers
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>

// Project Headers
#include "nbody.h"

// #define GRAPHICS
#ifdef GRAPHICS
	#include <SFML/Window.hpp>
	#include <SFML/Graphics.hpp>
#endif

// Number of particles
#define SMALL
//#define LARGE

#if defined(SMALL)
	const int N = 1000;
#elif defined(LARGE)
	const int N = 5000;
#endif

// Constants
const double min2 = 2.0;
const double G = 1 * 10e-10;
const double dt = 0.05;
const int NO_STEPS = 1000;
const int MAX_THREADS = 5; //N/10; //Max number of threads depends on total no bodies. A ration between cost savings and cost to make threads

// Size of Window/Output image
const int width = 1920;
const int height = 1080;
//int count = 0;

// Bodies
body bodies[N];

std::mutex queueMutex; //mutex to protect the jobqueue
//std::mutex cvMutex;
std::mutex terminateMutex;
//std::mutex other;
std::mutex accMutex;
std::queue<std::function<void()>> jobQueue; //std::function<void()> is a place holder for a random tbd function that the queue will hold
std::vector<std::thread> threads; //
std::condition_variable queueCV;
std::mutex  waitingThreadMut;
std::condition_variable innerLoopCV;

bool terminate = false;

void queueJob(std::function<void()> job){
	{
		//need to lock queue
		std::unique_lock<std::mutex> lockQueue(queueMutex);
		//if (jobQueue.size() == 0) std::cout << "\nbozo";
		jobQueue.push(job);
		//std::cout << "\nPushed Job"; 
	}
	queueCV.notify_one();
	//std::cout << "\nNotified queue"; 
}

void runTask(){
	//Each thread is always doing this. Will do something when there is a function in the queue
	std::cout << "\nTASK RUNNING";
	while(true){
		std::function<void()> currentJob;
		{
			//lock queue
			std::unique_lock<std::mutex> lockk(queueMutex);
//			if (jobQueue.size() == 0) std::cout << "\nTest2"; //runs 5 times
			//if(jobQueue.size()) std::cout << "\nEMPTY";
			//std::cout << "\nQUEUE LOCKED WAITING FOR JOB";
			queueCV.wait(lockk, [] (){
				return !jobQueue.empty() || terminate; 
			});
			//std::cout << "\nTHINGS GOIN";
//			if (jobQueue.size() == 0) std::cout << "\nTest3";
			if(terminate && jobQueue.empty()){ 
				return;
			}
			
			currentJob = std::move(jobQueue.front());
			jobQueue.pop(); 
			//std::cout << "\nSIZE: "<< jobQueue.size(); //crashing when it gets to zero?
		}
		queueCV.notify_one();
		currentJob();
	}
}

void createThreads(){
	for (size_t i = 0 ; i < MAX_THREADS; i++){
		std::thread thread1(runTask);
		threads.emplace_back(std::move(thread1));
	}
}

bool checkQueue(){ //checks jobqueue to see if there are any waiting / processing threads. If there are returns false
	{
		std::unique_lock<std::mutex> lock(queueMutex);
		return jobQueue.empty();
	}
}

void stopThreads(){
	//Stopping threads
	{
		std::unique_lock<std::mutex> lock(terminateMutex);
		terminate = true; //runTask accesses so have to protect
	}
	queueCV.notify_all();

	for(auto& thread : threads){
		thread.join();
	}
	threads.clear();
}

// Update Nbody Simulation
void update() { //what can llel???
	static size_t count = 0;
	count++;
	std::cout << "UPDATE No " << count << std::endl;
	// Acceleration
	vec2 acc[N];

	// Clear Acceleration
	for(int i = 0; i < N; ++i) { //Not needed
		acc[i] = vec2(0,0);
	}

	//For each update loop create an array of threads which will calculate bodyUpdate parallel to everything else

	// For each body
	for(int i = 0; i < N; ++i) {

		std::chrono::system_clock::time_point time1 = std::chrono::system_clock::now();
//		std::cout << "\nStarting Big Loop" << i;

		//THREADED SECTION
		// For each following body
		for(int j = i+1; j < N; ++j) {

//		if (j == i+1) std::cout << "\nStarting SMALL Loop" << j;

			//wait until thread is avaliable
			queueJob([&acc, i, j](){

				// Difference in position
				vec2 dx = bodies[i].pos - bodies[j].pos;

				// Normalised difference in position
				vec2 u = normalise(dx);

				// Calculate distance squared
				double d2 = length2(dx);
				
				// If greater than minimum distance
				if(d2 > min2) {
					// Force between bodies
					double f = -G*bodies[i].mass*bodies[j].mass / d2;

					std::lock_guard<std::mutex> lg(accMutex);
					// Add to acceleration
					acc[i] += (u * f / bodies[i].mass); //Okay to do threaded as not depended as value. HOWEVER CRIT SECTION SO PROTECT
					acc[j] -= (u * f / bodies[j].mass);
				}
			} );

			//std::cout << "\nHI";
		}

		{ //cv variable that stops loop continuing until all threads have finished
			std::unique_lock<std::mutex> waitLock(queueMutex);
//			std::cout << "\nNO";

			queueCV.wait(waitLock, [] (){
				return jobQueue.empty();
			});
			//queueCV.notify_one();
		}

//		std::chrono::system_clock::time_point time1b = std::chrono::system_clock::now();
			
		// 	//std::chrono::system_clock::time_point time2b = std::chrono::system_clock::now();
//		std::cout << "\nTime SMALL loop: " << std::chrono::duration_cast<std::chrono::microseconds>(time1b - time1).count()/1000000.0 << std::endl; 
	}

	// For each body
	// Put into thread????
	for(int i = 0; i < N; ++i) {
		// Update Position
		bodies[i].pos += bodies[i].vel * dt;

		// Update Velocity
		bodies[i].vel += acc[i] * dt;
	}
}

// 	// Threaded
// 	bodyUpdateArray[i] = std::thread([&acc, i] {
// 		// Update Position
// 		bodies[i].pos += bodies[i].vel * dt;

// 		// Update Velocity
// 		bodies[i].vel += acc[i] * dt;
// 	}); 

// //waiting for all threads to finish
// for(int i = 0 ; i < N ; i++){
// 	bodyUpdateArray[i].join();
// }

// Initialise NBody Simulation
void initialise() {
	// Create a central heavy body (sun)
	bodies[0] = body(width/2, height/2, 0, 0, 1e13, 5);

	// For each other body
	for(int i = 1; i < N; ++i) {
		// Pick a random radius, angle and calculate velocity
		double r = (uniform() + 0.1) * height/2;
		double theta = uniform() * 2 * M_PI;
		double v = (height) / r;

		// Create orbiting body
		bodies[i] = body(width/2 + r * cos(theta), height/2 + r * sin(theta), -sin(theta) * v, cos(theta)*v, 1e9, 1);
	}
}

#ifdef GRAPHICS
	// Main Function - Graphical Display
	int main() {
		// Create Window
		sf::ContextSettings settings;
		settings.antialiasingLevel = 1;
		sf::RenderWindow window(sf::VideoMode(width, height), "NBody Simulator", sf::Style::Default, settings);

		// Initialise NBody Simulation
		initialise();

		// run the program as long as the window is open
		while (window.isOpen()) {
			// check all the window's events that were triggered since the last iteration of the loop
			sf::Event event;
			while (window.pollEvent(event)) {
				// "close requested" event: we close the window
				if (event.type == sf::Event::Closed) {
					window.close();
				}
			}

			// Update NBody Simluation
			update();

			// Clear the window with black color
			window.clear(sf::Color::Black);

			// Render Objects
			for(int i = 0; i < N; ++i) {
				// Create Circle
				sf::CircleShape shape(bodies[i].radius);
				shape.setFillColor(sf::Color(255, 0, 0));
				shape.setPosition(bodies[i].pos.x, bodies[i].pos.y);
				
				// Draw Object
				window.draw(shape);
			}

			// Display Window
			window.display();
		}
	}
#else
	// Main Function - Benchmark
	int main() {
		// Initialise NBody Simulation
		initialise();

		// Get start time
		std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
		createThreads();
		// Run Simulation
		for(int i = 0; i < NO_STEPS; i++) { //Updating 1000 times
			// Update NBody Simluation
			update();
		}

		stopThreads();

		// Get end time
		std::chrono::system_clock::time_point end = std::chrono::system_clock::now();

		// Generate output image
		unsigned char *image = new unsigned char[width * height * 3];
		memset(image, 0, width * height * 3);

		// For each body
		for(int i = 0; i < N; ++i) {
			// Get Position
			vec2 p = bodies[i].pos;

			// Check particle is within bounds
			if(p.x >= 0 && p.x < width && p.y >= 0 && p.y < height) {
				// Add a red dot at body
				image[((((int)p.y * width) + (int)p.x) * 3)] = 255;
			}
		}

		// Write position data
		write_data("output.dat", bodies, N);

		// Write PNG output
		write_image("output.png", bodies, N, width, height);
		
		// Time Taken
		std::cout << "Time Taken: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()/1000000.0 << std::endl;
	}
#endif