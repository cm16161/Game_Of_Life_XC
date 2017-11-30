// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo code)

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"

#define  ImageSize 32      //define the dimensions of the image
#define  IMHT ImageSize    //image height
#define  IMWD ImageSize    //image width
#define  noWorkers 4       //defines the number of worker threads that process the image pixels

#define  steps 100          //defines how many rounds we do game of life for

#if(ImageSize % 32 == 0)   //macro that dynamically decides on size of bit packing internpretation
    #define IntSize 32     //if the image is large, aka greater than 32, interpret each pack as 32 bits
#else
    #define IntSize 16     //if the image size is small, pack into the first 16 bits of an int only
#endif

#define ubound ((IMHT * IMWD) / IntSize)  //defines the maximum number of ints in the world

typedef unsigned char uchar;      //using uchar as shorthand

on tile[0] : port p_scl = XS1_PORT_1E;         //interface ports to orientation
on tile[0] : port p_sda = XS1_PORT_1F;

on tile[0] : in port buttons = XS1_PORT_4E; //port to access xCore-200 buttons
on tile[0] : out port leds = XS1_PORT_4F;   //port to access xCore-200 LEDs

#define FXOS8700EQ_I2C_ADDR 0x1E  //register addresses for orientation
#define FXOS8700EQ_XYZ_DATA_CFG_REG 0x0E
#define FXOS8700EQ_CTRL_REG_1 0x2A
#define FXOS8700EQ_DR_STATUS 0x0
#define FXOS8700EQ_OUT_X_MSB 0x1
#define FXOS8700EQ_OUT_X_LSB 0x2
#define FXOS8700EQ_OUT_Y_MSB 0x3
#define FXOS8700EQ_OUT_Y_LSB 0x4
#define FXOS8700EQ_OUT_Z_MSB 0x5
#define FXOS8700EQ_OUT_Z_LSB 0x6

//function that routes a request to the LEDs
int showLEDs(out port p, chanend fromDist, chanend workerPause[noWorkers]) {
  int pattern = 0; //1st bit...separate green LED
               //2nd bit...blue LED
               //3rd bit...green LED
               //4th bit...red LED
  while (1) {
    select {   //receive a bit pattern from the different thread
        case workerPause[int i] :> pattern:  //conveniently pack the workers into one case with [int i]
            p <: pattern;  //dispatch the pattern
            break;
        case fromDist :> pattern:  //received from distributor
            p <: pattern;
            break;
    }
  }
  return 0;
}

void buttonListener(in port b, chanend toDist) { //connection to the button PINS on board
  int r;
  b when pinseq(14)  :> r;    //wait for exclusively SW1 to be pressed
  toDist <: 0; //send out that SW1 was pushed
  while (1) { //loop indefinitely: computation flows only based on SW2 now
      b when pinseq(13) :> r; //wait here until SW2 is pressed
      toDist <: 1; //send command to distributor
      toDist :> int data;
  }
}

//function that assigns the file to access based on img size
void assignFileName(char infname[20]) {
    int myImgSize = IMWD;
    char temp[10]; //need a temporary array to copy the image size into as a string
    sprintf(infname, "%d", myImgSize); //int -> string
    strcpy(temp, infname); //store conversion
    strcat(infname, "x"); //"(img)x"
    strcat(infname, temp); //"(img)x(img)"
    strcat(infname, ".pgm"); //(img)x(img).pgm"
}

//function that reads in the data from the file (executed once only)
void DataInStream(chanend c_out)
{
    char infname[20];          //get the target file name based on img size
    assignFileName(infname);

    int res;
    uchar line[ IMWD ];
    printf( "DataInStream: Start...\n" );
    res = _openinpgm( infname, IMWD, IMHT );

    //handle errors
    if( res ) {
      printf( "DataInStream: Error openening %s\n.", infname );
      return;
    }


    //read in the image values as pixels from the target file
    for( int y = 0; y < IMHT; y++ ) {
        _readinline( line, IMWD );
        for( int x = 0; x < IMWD; x++ ) {
            c_out <: line[ x ];
        }
    }

    _closeinpgm();
    printf( "DataInStream: Done...\n" );
    return;
}

//function that packs a pixel as an int into a single bit
void pack(chanend c_in, int world[ubound]){
  uchar buffer; //holds received value from file
  for( int y = 0; y < IMHT; y++ ) {               //go through all rows
      for( int x = 0; x < (IMWD/IntSize); x++ ) {           //go through each pixel per line
          for (int k = 0; k < IntSize; k++) {  //assign a value to every bit of the int
          c_in :> buffer;                      //read the pixel value
          if (buffer == 255) {                 //turn bit on for white, off for black
              world[(y*(IMWD/IntSize)) + x] = (world[(y*(IMWD/IntSize)) + x] | 1 << (IntSize -1 -k));
          }
        }
      }
  }
}

//function that checks if a particular bit in an int is on
int isAlive(int unit, int index){
    if (unit & (1 << (IntSize -1 - (index)))) return 1; //bit shift & masking to get the value of bit at an index
    else return 0;
}

//function that finds how many times the int_size goes into a particular column offset
int cleanDivide(int x) {
    int counter = 0;
    int remainder = x;
    while (remainder >= IntSize) {  //loop while we can still divide
        counter = counter + 1;
        remainder = remainder - IntSize; //account for finding another division
    }
    return counter;
}

//function that checks how many cells are alive in a worker's world
int liveCellCount(int outputworld[ubound/noWorkers]){
    int liveCell = 0;
    for (int y = 0; y< IMHT/noWorkers; y++){ //loop for all rows
        for(int x =0; x<(IMWD/IntSize); x++){ //loop for every column (offset in the row)
            for (int k = 0; k < IntSize; k++) { //loop across each bit in the offset
                if(isAlive(outputworld[(y*(IMWD/IntSize)) + x],k%IntSize)){ //check if bit is on
                     liveCell++;
                }
            }
       }
    }
return liveCell;
}

//function to get the number of living neighbours for a given index x in the current row being processed
int getLiveNeighbours(int index, int processArray[3*(IMWD/IntSize)], int top, int bottom , int mid, int x){
int counter = 0;
for(int j = -1; j<2; j++){ //loop for the 3 surrounding columns
    for(int i =-1; i<2; i++){ //loop for the 3 surrounding rows
        int xneighbour = (x+i) ;           //get the row offset
        if(xneighbour == -1) xneighbour = IMWD -1;
        else if(xneighbour == IMWD) xneighbour = 0;
        //calculate which row we would like to target
        int yneighbour;
        if(j == -1)yneighbour = top;
        else if(j== 1) yneighbour = bottom;
        else yneighbour = mid;
        //make sure that we dont include the original square
        if((i != 0) || (j!=0)){
            int offset = cleanDivide(xneighbour); //get the offset for x
            //check if the target pixel is alive, if so, increment the counter
            if(isAlive(processArray[(yneighbour*(IMWD/IntSize))+offset], xneighbour % IntSize)) counter++;
        }
    }
}
return counter;
}

//function that sends the output from worker to distributor
void sendOutput(chanend toDist, int outputworld[ubound/noWorkers]){
    for (int y = 0; y<IMHT/noWorkers;y++){ //loop across every row
        for (int x = 0; x < IMHT/IntSize; x++) { //loop across every column offset
            toDist <: outputworld[(y*(IMWD/IntSize)) + x]; //send the int corresponding to 32 or 16 img pixels
        }
    }
}

//receive the next part of the image to process from the worker
void recieveWork(chanend toDist, int processArray[3*(IMWD/IntSize)], int overWriteIndex){
    for (int x = 0; x <(IMWD/IntSize); x++){ //receive the whole row
        //overwrite the oldest part of the 3-row array (using overWriteIndex)
        toDist :> processArray[overWriteIndex * (IMWD/IntSize) + x];
    }
}

//initialise the arrays in the worker
void initialiseArrays(int outputworld[ubound/noWorkers], int processArray[3*(IMHT/IntSize)]){
    for (int y = 0; y<IMHT/noWorkers;y++){ //loop for every row in the world
         for (int x = 0; x < IMHT/IntSize; x++) { //loop across the whole row
              outputworld[(y*(IMWD/IntSize)) + x] = 0;
         }
    }
    for (int y = 0; y < 3; y++) { //we only ever store 3 rows at a time in the process array
        for (int x = 0; x < (IMHT/IntSize); x++) { //loop across whole row
            processArray[(y*(IMHT/IntSize)) + x] = 0;
        }
    }
}

//function that changes a particular bit in an int
void changeBit(int outputworld[ubound/noWorkers], int outputWorldUnit, int x, int change){
    //if we are changing to a 1, we need to or it, in case the target bit is 0
    if (change == 1) {
        outputworld[outputWorldUnit] = outputworld[outputWorldUnit] | (change << IntSize -1 - x);
    }
    //if changing to a 0, we & the current bit with 0. also need to convert other parts to a 1 to preserve values.
    else {
        outputworld[outputWorldUnit] = outputworld[outputWorldUnit] & ~(change << IntSize -1 - x);
    }
}

//function that processes a row of the output world
void transformPixel(int index, int processArray[3*(IMWD/IntSize)],int depth, int mid, int top, int bottom, int outputworld[ubound/noWorkers]){
        for(int x =0; x < IMWD; x++){ //loop for all bits in the row
          int offset = cleanDivide(x); //get the offset in the row
          int liveNeighbours = getLiveNeighbours(depth, processArray,top,bottom, mid,x); //get neighbours of bit
          int isLiving = isAlive(processArray[(mid * (IMWD/IntSize)) + offset], x % IntSize); //check if this bit is on
          //rule implementation for game of life-> condensed version using case summation
          if ((isLiving == 1 && liveNeighbours == 3) || liveNeighbours + isLiving == 3) {
              changeBit(outputworld, (depth*(IMWD/IntSize)) + offset, x%IntSize, 1); //switch the bit on
          } else {
              changeBit(outputworld, (depth*(IMWD/IntSize)) + offset, x%IntSize, 0); //turn it off
          }
        }
}

//functin that performs the shifting of the indices in the proces array
{int, int, int, int, int} shiftIndices(int depth, int mid, int top, int bottom, int overWritePart) {
    depth = depth + 1;  //depth in outputworld increases by one
    mid = (mid + 1) % 3;  //shift indices by one
    top = (top + 1) % 3;
    bottom = (bottom + 1) % 3;
    overWritePart = (overWritePart + 1) % 3; //get new overwrite row
    return {depth, mid, top, bottom, overWritePart};
}

void pauseWorkerManager(float time, chanend pauseWorker, int pause, int index, int outputworld[ubound/noWorkers], int rounds, chanend toLEDs, int alternator) {
    pauseWorker :> pause;
    if(pause ==1){
        printf("Number of Live Cells Processed by Worker %d: %d\n",index, liveCellCount(outputworld));
        if(index == 0) {
            printf("Time elapsed is: %f\n", time/(float)1000000000);
            printf("Number of Rounds Processed: %d\n", (rounds));
        }
        while (pause == 1) {
            pauseWorker :> pause;
            toLEDs <: 8 | alternator;
        }
    }
}

void processWorker(chanend toDist,chanend pauseWorker, chanend toLEDs, int index, chanend toTimer){
  int counter = steps, pause = 0, alternator =0, rounds =0;
  float time = 0;
  while(counter >0){
    toDist :> alternator;
    int processArray[3*(IMWD/IntSize)], outputworld[ubound/noWorkers];
    int top = 0, mid = 1, bottom = 2, depth = 0, overWritePart = 2;
    initialiseArrays(outputworld, processArray);
    recieveWork(toDist, processArray, 0);
    recieveWork(toDist, processArray, 1);
    for (int y = 0; y<(IMHT/noWorkers);y++){
        pauseWorkerManager(time, pauseWorker, pause, index, outputworld, rounds,toLEDs,alternator);
        pause = 0;
        toLEDs <: 0 | alternator;
        recieveWork(toDist, processArray, overWritePart);
        transformPixel(index, processArray, depth, mid, top, bottom, outputworld);
        {depth, mid, top, bottom, overWritePart} = shiftIndices(depth, mid, top, bottom, overWritePart);
    }
    rounds++;
    sendOutput(toDist, outputworld);
    toTimer :> time;
}
}

//function that sends out the image part belonging to a worker
void distributeWork(chanend worker[noWorkers], int world[ubound]){
  par (int index = 0; index < noWorkers; index ++){ //in parallel, all workers get their part of the image
      // loop across the rows belinging to a worker: for (int base; int interval + base; i = i+1)
      for(int y = ((index*IMHT)/noWorkers)- 1 ; y < (((index*IMHT)/noWorkers) + IMHT/noWorkers) + 1; y++){
          int rowCalc = y; //set target round
          if (rowCalc == -1) rowCalc = IMHT-1; //wrap around
          if (rowCalc == IMHT) rowCalc = 0;
          for (int x = 0; x < (IMWD/IntSize);x++) { //distribute all packed row parts to worked
            worker[index] <: world[(rowCalc*(IMWD/IntSize))+x];
          }
        }
      }
}

//receive the processed part of the image from the worker
void recieveFinal(chanend worker[noWorkers], int world[ubound]){
    for(int index =0; index<noWorkers;index++){ //sequentially receive in order
        int base = index*(IMHT/noWorkers); //calculate the base of where in the world to place
        int interval = base + (IMHT/noWorkers); //calculate the upper limit, aka the index of the last pixel received
        for(int y = base; y< interval; y++){ //loop across all rows
            for(int x = 0; x < (IMWD/IntSize); x++){ //loop across all packed parts of the row
                worker[index] :> world[(y*(IMWD/IntSize)) + x];
            }
        }
    }
}

//holds program until the SW1 button is pushed
void getButton(chanend fromButtons, chanend display, int alternator){
  fromButtons :> int buttonPress;
  display <: 4 | alternator; //display the green LED while loading in the image
}

void sendAlternator(int alternator, chanend worker[noWorkers]){
  //set the alternator (aka dark green flashing LED)
  if (alternator == 1) alternator = 0;
  else alternator = 1;

  //send the alternator to the workers
  for (int a = 0; a < noWorkers; a++) {
      worker[a] <: alternator;
  }
}

//unpacks the bit representation of the image into uchars, as required by data out
void unpack(int world[ubound], chanend c_out){
    uchar auth = 1; //authorisation is granted to open the file to write data (prevents corruption of file)
    c_out <: auth; //send authorsation to data out
    for (int y = 0; y< IMHT; y++){ //loop across the rows
        for(int x =0; x<(IMWD/IntSize); x++){ //loop across all offsets in the packed row
            for (int k = 0; k < IntSize; k++) { //loop across each bit
                uchar temp; //assign a uchar for communication
                if(isAlive(world[(y*(IMWD/IntSize)) + x],k%IntSize)){
                     temp = 255; //if the particualr bit is a 1, send a 255 (white)
                }
                else temp = 0; //otherwise send a 0 (dead aka black)
                c_out <: temp; //send info
            }
       }
    }
}

//function that initialised the world
void initialiseWorld(int world[ubound]){
    for (int i = 0; i < ubound; i++) {
          world[i] = 0; //set all elements to a 0
      }
}

//manages the blue LED/ output button
void buttonManager(chanend c_out, chanend fromButtons, chanend toLEDs, int receival, int alternator, int counter, int world[ubound]) {
    select {
        case fromButtons :> receival: //command received from the button
            toLEDs <: 2 | alternator; //flash blue
            unpack(world,c_out); //output the data
            toLEDs <: 0 | alternator;
            break;
        default:
            toLEDs <: 0 | alternator; //keeo the LED off
            //if we're on the last round, for convenience output the world
            if(counter == 1) {toLEDs <: 2 | alternator; unpack(world, c_out); toLEDs <: 0 | alternator;}
            break;
    }
}

void distributor(chanend toLEDs, chanend c_in, chanend c_out, chanend fromAcc, chanend worker[noWorkers], chanend fromButtons, chanend toTimer)
{
  int counter = steps; //control the loop
  float time = 0;
  float oneSecond = 100000000;
  int alternator = 0;
  int world[ubound];
  int receival = 1;
  printf("Waiting for Button Click...\n");
  initialiseWorld(world); //initialise the main world array
  getButton(fromButtons, toLEDs, alternator); //wait until the green button is pressedx
  pack(c_in, world); //pack the input into a condensed world
  toLEDs <: 0 | alternator; //clear the LEDs

  while (counter > 0) { //loop for all specified steps
      printf( "Processing...\n" );
      toTimer <: 1; //activate the timer
      sendAlternator(alternator, worker); //send the alternator LED command
      distributeWork(worker, world); //send the work to the workers
      recieveFinal(worker,world); //receive the work from the workers
      toTimer <: 0; //halt the timer
      toTimer :> time; //get the time
      if (counter == 1) printf("THE TIME DIFFERENCE WAS %lf SECONDS\n", time/oneSecond);
      buttonManager(c_out, fromButtons, toLEDs, receival, alternator, counter, world); //check for write request
      counter = counter - 1;
      printf( "\nOne processing round completed...\n" );
  }
  uchar auth = 0; //refuse authorisation to open file for next round
  c_out <: auth;
}

void recordTime(chanend toDist, chanend toWorker[noWorkers]){
    float total = 0;
    unsigned int int_max = 4294967295;
    unsigned int getTime = 0, running = 1, baseTime = 0, previous = 0, period = 1000000000;
    timer t;
    while(1){
        toDist :> running;
        t :> getTime;
        baseTime = getTime;
        previous = baseTime;
        select {
            case t when timerafter(getTime + period) :> void:
                t :> getTime;
                if (getTime < previous){
                    float tempContainer = (int_max - previous);
                     total = total + (tempContainer - getTime);
                } else {
                    total = total + period;
                }
                previous = getTime;
                break;
            case toDist :> running:
                t :> getTime;
                if(getTime < previous){
                    float tempContainer = (int_max - previous);
                    total = total + (tempContainer - getTime);
                } else {
                    total = total + (float)(getTime - previous);
                }
                previous = getTime;
                toDist <: total;

                break;
        }
        for (int i = 0; i<noWorkers; i++){
            toWorker[i] <: total;
        }
   }
}

void DataOutStream(chanend c_in)
{
  int res;
  char outfname[20] = "junkk.pgm"; //put your output image path here
  uchar distAuth = 1; //authorisation to write from the distributor
  while (1){
      uchar line[ IMWD ];
      c_in :> distAuth; //receive an authorisation to open the file. given when writing needed.
      if (distAuth == 1){
          printf( "DataOutStream: Start...\n" );
          res = _openoutpgm( outfname, IMWD, IMHT );
      }
      if( res ) { //deal with error
          printf( "DataOutStream: Error opening %s\n.", outfname );
          return;
      }
      //Compile each line of the image and write the image line-by-line
      for( int y = 0; y < IMHT; y++ ) {
          for( int x = 0; x < IMWD; x++ ) {
              c_in :> line[ x ];
          }
          _writeoutline( line, IMWD );
          printf( "DataOutStream: Line written...\n" );
      }

      //Close the PGM image
      _closeoutpgm();
      printf( "DataOutStream: Done...\n" );

  }
  return;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Initialise and  read orientation, send first tilt event to channel
//
/////////////////////////////////////////////////////////////////////////////////////////
void orientation( client interface i2c_master_if i2c, chanend toDist, chanend pauseWorker[noWorkers]) {
  i2c_regop_res_t result;
  char status_data = 0;
  int tilted = 0;

  // Configure FXOS8700EQ
  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_XYZ_DATA_CFG_REG, 0x01);
  if (result != I2C_REGOP_SUCCESS) {
    printf("I2C write reg failed\n");
  }

  // Enable FXOS8700EQ
  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_CTRL_REG_1, 0x01);
  if (result != I2C_REGOP_SUCCESS) {
    printf("I2C write reg failed\n");
  }

  //Probe the orientation x-axis forever
  while (1) {

    //check until new orientation data is available
    do {
      status_data = i2c.read_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_DR_STATUS, result);
    } while (!status_data & 0x08);

    //get new x-axis tilt value
    int x = read_acceleration(i2c, FXOS8700EQ_OUT_X_MSB);

    //send signal to distributor after first tilt

    if (!tilted){
      if(x <= -25){
        tilted =  1;
        for (int i = 0; i < noWorkers; i++) {
          pauseWorker[i] <: 1;
        }
      }
      else {
        for (int i = 0; i < noWorkers; i++) {
          pauseWorker[i] <: 0;
        }
      }
    }
    else {
      if(x >= 25){
        tilted = 0;
        for (int i = 0; i< noWorkers; i++){
          pauseWorker[i] <: 0;
        }
      }
      else {
        for (int i = 0; i<noWorkers; i++){
          pauseWorker[i] <: 1;
        }
      }
    }
    }
  }


/////////////////////////////////////////////////////////////////////////////////////////
//
// Orchestrate concurrent system and start up all threads
//
/////////////////////////////////////////////////////////////////////////////////////////
int main(void) {

    i2c_master_if i2c[1];               //interface to orientation

    chan c_inIO, c_outIO, c_control;  //channels for IO
    chan worker[noWorkers];           //channels between distributor and the workers
    chan buttonCom;                   //communication between distributor and the buttons
    chan pauseWorker[noWorkers];      //channels to pause the worker on a tilt
    chan LEDWorkerComm[noWorkers];    //channels to light the LEDs fron the workers
    chan distTimer;                   //channels to access timer from distributor
    chan LEDsDist;                    //distributor LED channel
    chan workerTimer[noWorkers];

    par {
        on tile[0] : i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing orientation data
        on tile[1] : orientation(i2c[0],c_control, pauseWorker);        //client thread reading orientation data
        on tile[0] : DataInStream(c_inIO);          //thread to read in a PGM image
        on tile[0] : DataOutStream(c_outIO);       //thread to write out a PGM image
        on tile[1] : distributor(LEDsDist, c_inIO, c_outIO, c_control, worker, buttonCom, distTimer);//thread to coordinate work on image
        on tile[0] : buttonListener(buttons, buttonCom); //button port thread
        on tile[0] : showLEDs(leds, LEDsDist, LEDWorkerComm); //LED port thread
        on tile[0] : recordTime(distTimer, workerTimer); //timer thread

        //run a number of workers in parallel
        par(int index = 0; index<noWorkers; index++){
            on tile[index % 2] : processWorker(worker[index], pauseWorker[index], \
                    LEDWorkerComm[index], index, workerTimer[index]);
        }
    }
  return 0;
}
