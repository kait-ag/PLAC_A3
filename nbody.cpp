// System Headers
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <queue>

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
	const int N = 500;
#elif defined(LARGE)
	const int N = 5000;
#endif

// Constants
const double min2 = 2.0;
const double G = 1 * 10e-10;
const double dt = 0.05;
const int NO_STEPS = 1000;
const int MAX_THREADS = 3; //N/10; //Max number of threads depends on total no bodies. A ration between cost savings and cost to make threads

// Size of Window/Output image
const int width = 1920;
const int height = 1080;

// Bodies
body bodies[N];
std::mutex coutMut;
//std::mutex test;

class ThreadPool{
	public:
	explicit ThreadPool(const std::size_t threadNum);
	void queueJob(std::function<void()> job);

	private:
	void runTask();
	std::mutex queueMutex;
	std::queue<std::function<void()>> jobQueue; //std::function<void()> is a place holder for a random tbd function that the queue will hold
	std::vector<std::thread> threads; //Holds all the treads currently running
	std::condition_variable queueCV;
};

ThreadPool::ThreadPool(const std::size_t threadNum){
	for (size_t i = 0; i < threadNum ; i++){
		threads.emplace_back(std::thread(runTask));
	}
}

void ThreadPool::queueJob(std::function<void()> job){
	{
		//need to lock queue
		std::unique_lock<std::mutex> lockQueue(queueMutex);
		jobQueue.push(job); //pushes job into queue
	}
	queueCV.notify_one();

}

void ThreadPool::runTask(){
	//Always running! will do something when there is a function in the queue
	while(true){
		std::function<void()> currentJob;
		{
			//lock queue
			std::unique_lock<std::mutex> lock(queueMutex);
			queueCV.wait(lock, [this] {
				return !jobQueue.empty(); //Waits for jobQueue to return NOT empty
			});
			currentJob = jobQueue.front();
			jobQueue.pop();
		}
		currentJob(); //then run currentJob;
	}
}

ThreadPool threadPool(MAX_THREADS); //creates a new threadPool class

// Update Nbody Simulation
void update() {

	// Acceleration
	vec2 acc[N];

	// Clear Acceleration
	for(int i = 0; i < N; ++i) { //Not needed
		acc[i] = vec2(0,0);
	}

	//For each update loop create an array of threads which will calculate bodyUpdate parallel to everything else
	std::mutex protectMutex; //Protects shared acc[] 
	std::mutex controlMutex; //Used to control cv.wait()
	std::mutex threadCountMutex; //Protects the threadCount
	std::condition_variable cv;

	std::chrono::system_clock::time_point time1 = std::chrono::system_clock::now();
	// For each body
	for(int i = 0; i < N; ++i) {

		size_t activeThreads = 0;

		std::cout << "\nHI" ;

		//THREADED SECTION
		// For each following body
		for(int j = i+1; j < N; ++j) {

			threadPool.queueJob([&acc, i, j, &protectMutex, &threadCountMutex, &cv, &activeThreads, &controlMutex]() -> void {
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

					std::lock_guard<std::mutex> lg(protectMutex); //Protects acc[]
					// Add to acceleration
					acc[i] += (u * f / bodies[i].mass); //Okay to do threaded as not depended as value. HOWEVER CRIT SECTION SO PROTECT
					acc[j] -= (u * f / bodies[j].mass);
				}

				threadCountMutex.lock();
				if(activeThreads != 0) activeThreads--;
				threadCountMutex.unlock();
				cv.notify_one(); //notifies another thread that it can continue
			});

		}

		std::chrono::system_clock::time_point time1b = std::chrono::system_clock::now();

		coutMut.lock();
		std::cout << "\n End of I loop " << i;
		std::cout << "\nactive threads = " << activeThreads;

		// std::unique_lock<std::mutex> finishedThreadLock(controlMutex);
		// cv.wait(finishedThreadLock, [&activeThreads]() {return activeThreads == 0;}); //Waits until all activeThreads are finished

		std::cout << "\nTime Big loop: " << std::chrono::duration_cast<std::chrono::microseconds>(time1b - time1).count()/1000000.0 << std::endl; 
		coutMut.unlock();

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