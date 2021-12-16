# Motion_Planning_Final
Final Project for ASEN5519


Directions For Running:

The planning programs used for this project are all found in: ompl-1.5.2/asen5519_final_dev. 

To compile code, first make sure you have OMPL installed: https://ompl.kavrakilab.org/installation.html

It is recommended you merge this repository into the file structure of your installed OMPL. I have made several changes to src code that may cause compilation errors if they do not match the changes on this repo.

Once this is done, navigate to ompl-1.5.2/build/Release. Execute the command: 'make'. If this does not build the planning programs, then try the cmake command: "cmake -DOMPL_BUILD_FINALPROJ=ON ../..", then attempt to make again.

Navigate to the bin folder. Each planner executable exists in their with the preface: "mp_". To run the sst planner for instance, execute the command: "./mp_Kinodynamic_SST_UAV". Results are automatically printed to text files inside that same bin folder. The names of these text files should clearly correspond to the planner that generated them.


Directions for processing:
Copy the text file you wish to process to the Motion_Planning_Final/results/ folder. Run the visualization.m file to see results. There are several parameters at the beginning of the file which will change the formatting of results.
