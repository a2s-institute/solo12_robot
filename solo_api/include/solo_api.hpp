#ifndef SOLO_API_H
#define SOLO_API_H

// #include "master-board/sdk/master_board_sdk/include/master_board_sdk/master_board_interface.h"
// #include "master-board/sdk/master_board_sdk/include/master_board_sdk/defines.h"

class SOLO_API{
public:
    // Constructor
    SOLO_API(int *argc, char ***argv); //allowing both type of input

    // Destructor
    ~SOLO_API();

    // Member functions
    int initialize();
    int performAction();

    // Member variables
    int data;
    // MasterBoardInterface robot_if(std::string if_name);


private:
    // Private member variables
    double value;
};

#endif // SOLO_API_H
