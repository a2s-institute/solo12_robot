#ifndef SOLO_API_H
#define SOLO_API_H

class SOLO_API {
public:
    // Constructor
    SOLO_API();

    // Destructor
    ~SOLO_API();

    // Member functions
    void initialize();
    void performAction();

    // Member variables
    int data;

private:
    // Private member variables
    double value;
};

#endif // SOLO_API_H
