# solo12_robot

Base code of the controller of the Nala robot (SOLO12 robot from exisiting examples from the master-board/sdk/masterboard-sdk/examples/)

### Required to do

[-]Use the existing example files as controllers in the abstarct layer by calling them as a library file


### Notes

*For Basic Understandings of structure of C++*

- The constructor and desturctor is used to run the class only when it is required,
- But, in order to save the memory space, we can use the destructor to delete or erase the constructor to free the space
- Inside the class we will be creating a private keyword, which are only accessible within the class
	- Whereas, Public keywords allows to call the functions or varaibles, even outside the class
