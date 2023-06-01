// System Headers
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>

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
	const int N = 10;
#elif defined(LARGE)
	const int N = 5000;
#endif

// Constants
const double min2 = 2.0;
const double G = 1 * 10e-10;
const double dt = 0.05;
const int NO_STEPS = 10;

// Size of Window/Output image
const int width = 1920;
const int height = 1080;

// Bodies
body bodies[N];
std::mutex coutMut;


// Update Nbody Simulation
void update() { //what can llel???

	// Acceleration
	vec2 acc[N];

	// Clear Acceleration
	for(int i = 0; i < N; ++i) { //Not needed
		acc[i] = vec2(0,0);
	}

	//For each update loop create an array of threads which will calculate bodyUpdate parallel to everything else
	//std::thread bodyUpdateArray[N];
	//std::vector<std::thread> innerLoopThread;
	std::mutex protectMutex; //
	std::mutex controlMutex;
	std::mutex threadCountMutex;
	std::condition_variable cv;

	const int MAX_THREADS = 10; //N/10; //Max number of threads depends on total no bodies. A ration between cost savings and cost to make threads

	std::chrono::system_clock::time_point time1 = std::chrono::system_clock::now();
	// For each body
	for(int i = 0; i < N; ++i) {
		size_t activeThreads = 0;

		//THREADED SECTION
		// For each following body
		for(int j = i+1; j < N; ++j) {
			coutMut.lock();
			coutMut.unlock();
			//wait until thread is avaliable
			std::unique_lock<std::mutex> lock(controlMutex);
			cv.wait(lock, [&activeThreads, MAX_THREADS]() {return activeThreads < MAX_THREADS;}); //NEED TO PROTECT ACTIVE THREAD
			
			threadCountMutex.lock();
			activeThreads++;
			threadCountMutex.unlock();
			std::cout << "\nStarted thread" << activeThreads;
			lock.unlock();

			std::thread thread1([&acc, i, j, &protectMutex, &threadCountMutex, &cv, &activeThreads, &lock](){

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

					std::lock_guard<std::mutex> lg(protectMutex);
					// Add to acceleration
					acc[i] += (u * f / bodies[i].mass); //Okay to do threaded as not depended as value. HOWEVER CRIT SECTION SO PROTECT
					acc[j] -= (u * f / bodies[j].mass);
				}
				//PROTECT MUTEX

				//lock.lock();
				threadCountMutex.lock();
				activeThreads--; //Protect??
				std::cout << "\nThread finished: " << activeThreads;
				threadCountMutex.unlock();

				//lock.unlock();

				coutMut.lock();
				std::cout << "\nUNLOCKED";
				coutMut.unlock();

				cv.notify_one(); //notifies a thread that it can continue
			});
			thread1.detach();
		}

		std::chrono::system_clock::time_point time1b = std::chrono::system_clock::now();

		//Ensure all the inner loop threads are finished
		// for (auto &thread : innerLoopThread){
		// 	if(thread.joinable()){
		// 		thread.join();
		// 	}
		// }

	std::unique_lock<std::mutex> finishedThreadLock(controlMutex);
	cv.wait(finishedThreadLock, [&activeThreads]() {return activeThreads == 0;});

	// 	// Threaded
	// 	bodyUpdateArray[i] = std::thread([&acc, i] {
	// 		// Update Position
	// 		bodies[i].pos += bodies[i].vel * dt;

	// 		// Update Velocity
	// 		bodies[i].vel += acc[i] * dt;
	// 	}); 
		
	// 	//std::chrono::system_clock::time_point time2b = std::chrono::system_clock::now();
	std::cout << "\nTime Big loop: " << std::chrono::duration_cast<std::chrono::microseconds>(time1b - time1).count()/1000000.0 << std::endl; 
	// 	}


	// innerLoopThread.clear();
	
	// //waiting for all threads to finish
	// for(int i = 0 ; i < N ; i++){
	// 	bodyUpdateArray[i].join();
	// }

	// For each body
	// Put into thread????
	for(int i = 0; i < N; ++i) {
		// Update Position
		bodies[i].pos += bodies[i].vel * dt;

		// Update Velocity
		bodies[i].vel += acc[i] * dt;
	}
	}
}

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

		// Run Simulation
		for(int i = 0; i < NO_STEPS; i++) { //Updating 1000 times
			// Update NBody Simluation
			update();
		}

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