// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo code)

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"

#define  ImageSize 128
#define  IMHT ImageSize                  //image height
#define  IMWD ImageSize                  //image width
#define  noWorkers 4

#define  steps 1

#if(ImageSize % 32 == 0)
    #define IntSize 32
#else
    #define IntSize 16
#endif

#define ubound ((IMHT * IMWD) / IntSize)

typedef unsigned char uchar;      //using uchar as shorthand

on tile[0] : port p_scl = XS1_PORT_1E;         //interface ports to orientation
on tile[0] : port p_sda = XS1_PORT_1F;

char infname[20] = "128x128.pgm";     //put your input image path here
char outfname[20] = "testoutNOOT128.pgm"; //put your output image path here

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

int showLEDs(out port p, chanend fromButtons, chanend workerPause[noWorkers]) {
  int pattern = 0; //1st bit...separate green LED
               //2nd bit...blue LED
               //3rd bit...green LED
               //4th bit...red LED
  while (1) {
    select {
        case fromButtons :> pattern:
            p <: pattern;
            break;
        case workerPause[int i] :> pattern:
            p <: pattern;
            break;
    }
  }
  return 0;
}

void buttonListener(in port b, chanend toDist, chanend toLEDs) {
  int r;
  int data;
  int writtenback = 1;

  while (1) {
      b when pinseq(15)  :> r;    // check that no button is pressed
      b when pinsneq(15) :> r;    // check if some buttons are pressed
      if (r == 14 && writtenback != 0) {
          toDist <: 0; //send out that a button was pushed

          toDist :> data; //confirmation of recepit = green light
          toLEDs <: data; //display green light

          toDist :> data; //terminate green light at the end of processing
          toLEDs <: data;
          writtenback = 0;
      }
      if (r == 13 && writtenback == 0) {
          toDist :> data;
          toLEDs <: data;
          toDist :> data;
          toLEDs <: data;
          writtenback = 1;
      }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Read Image from PGM file from path infname[] to channel c_out
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataInStream(char infname[], chanend c_out)
{
  int counter = steps;
  int alternator = 0;
  while (counter >0){
    int res;
    uchar line[ IMWD ];
    printf( "DataInStream: Start...\n" );

    //Open PGM file
    res = _openinpgm( infname, IMWD, IMHT );
    if( res ) {
      printf( "DataInStream: Error openening %s\n.", infname );
      return;
    }

  //Read image line-by-line and send byte by byte to channel c_out
  for( int y = 0; y < IMHT; y++ ) {
    _readinline( line, IMWD );
    for( int x = 0; x < IMWD; x++ ) {
      c_out <: line[ x ];
//      if (line[x] == 255)printf("1");
  //    else printf("0");
    }
//    printf("\n");
  }

  //Close PGM image file
  _closeinpgm();
  printf( "DataInStream: Done...\n" );
  counter = counter - 1;
  if (alternator == 0) {
      strcpy(infname,"testout.pgm");
      alternator = 1;
  }
  c_out :> int data;
}
  return;
}


void pack(chanend c_in, int world[ubound]){
  uchar buffer;
  for( int y = 0; y < IMHT; y++ ) {               //go through all lines
      for( int x = 0; x < (IMWD/IntSize); x++ ) {           //go through each pixel per line
          for (int k = 0; k < IntSize; k++) {
          c_in :> buffer;                      //read the pixel value
          if (buffer == 255) {
              world[(y*(IMWD/IntSize)) + x] = (world[(y*(IMWD/IntSize)) + x] | 1 << (IntSize -1 -k));
          }
        }
      }
  }
}

int isAlive(int unit, int index){
    if (unit & (1 << (IntSize -1 - (index)))) return 1;
    else return 0;
}

int cleanDivide(int x) {
    int counter = 0;
    int remainder = x;
    while (remainder >= IntSize) {
        counter = counter + 1;
        remainder = remainder - IntSize;
    }
    return counter;
}

int getLiveNeighbours(int index, int processArray[3*(IMWD/IntSize)], int top, int bottom , int mid, int x){
int counter = 0;
int flagup, flagdown;
for(int j = -1; j<2; j++){
    for(int i =-1; i<2; i++){
        flagup = 0;
        flagdown = 0;
        int xneighbour = (x+i) ;
        int yneighbour;
        if(xneighbour == -1) xneighbour = IMWD -1;
        if(xneighbour == IMWD) xneighbour = 0;

        if(j == -1) {yneighbour = top; flagup =1;}
        else if(j== 1) {yneighbour = bottom; flagdown = 1;}
        else if(j == 0) {yneighbour = mid;}
        if((i != 0) || (j!=0)){
            int passTop = cleanDivide(xneighbour);
            if(flagup == 1){
                if(isAlive(processArray[passTop], xneighbour % IntSize)) counter++;
                flagup =0;
            }
            else if(flagdown == 1){
                int passBottom = cleanDivide(xneighbour);
                if(isAlive(processArray[(2*(IMWD/IntSize))+passBottom], xneighbour % IntSize)) counter++;
                flagdown =0;
            }
            else{
                int passY = cleanDivide(xneighbour);
                if(isAlive(processArray[((IMWD/IntSize))+passY], xneighbour % IntSize)) counter++;
            }
        }
    }
}
//printf("$%d\n", counter);

return counter;
}

void sendOutput(chanend toDist, int outputworld[ubound/noWorkers]){
    for (int y = 0; y<IMHT/noWorkers;y++){
        for (int x = 0; x < IMHT/IntSize; x++) {
            if((y==5) && (x==0)){
            }
            toDist <: outputworld[(y*(IMWD/IntSize)) + x];
        }
    }
}

void recieveOverlap(chanend toDist, int processArray[3*(IMWD/IntSize)]){
    for (int x = 0; x <(IMWD/IntSize); x++){
        toDist :> processArray[x];
        toDist :> processArray[(IMWD/IntSize)+x];
        toDist :> processArray[2*(IMWD/IntSize)+x];
    }
}

/*void recieveWork(chanend toDist, int processArray[3*(IMWD/IntSize)], int overWriteIndex){
            for (int x = 0; x < IMWD/IntSize; x++) {
                toDist :> processArray[(overWriteIndex*(IMWD/IntSize)) + x]; //packed up row...
            }
       // }
}*/

void initialiseOutput(int outputworld[ubound/noWorkers], int processArray[3*(IMHT/IntSize)]){
    for (int y = 0; y<IMHT/noWorkers;y++){
         for (int x = 0; x < IMHT/IntSize; x++) {
              outputworld[(y*(IMWD/IntSize)) + x] = 0;
         }
    }
    for (int y = 0; y < 3; y++) {
        for (int x = 0; x < (IMHT/IntSize); x++) {
            processArray[(y*(IMHT/IntSize)) + x] = 0;
        }
    }
}

void pauseWorkerFunction(chanend pauseWorker, int pause, chanend toLEDs, int alternator){
  pauseWorker :> pause;
  while (pause == 1) {
      pauseWorker :> pause;
      toLEDs <: 8 | alternator;
  }
  toLEDs <: 0 | alternator;
}

void changeBit(int outputworld[ubound/noWorkers], int outputWorldUnit, int x, int change){
    if (change == 1) {
        outputworld[outputWorldUnit] = outputworld[outputWorldUnit] | (change << IntSize -1 - x);
    }
    else {
        outputworld[outputWorldUnit] = outputworld[outputWorldUnit] & ~(change << IntSize -1 - x);
    }
}

void transformPixel(int index, int processArray[3*(IMWD/IntSize)],int depth, int mid, int top, int bottom, int outputworld[ubound/noWorkers], chanend pauseWorker, int pause, chanend toLEDs, int alternator){
    int liveNeighbours;
    int counter = 0;
        for(int x =0; x < IMWD; x++){
          counter = counter + 1;
          pauseWorkerFunction(pauseWorker, pause, toLEDs, alternator);
          int offset = cleanDivide(x);
          liveNeighbours = getLiveNeighbours(depth, processArray,top,bottom, mid,x);     //////////THIS NEEDS TO BE FIXED
          int isLiving = isAlive(processArray[((IMWD/IntSize)) + offset], x % IntSize);
          if(isLiving){
            if(liveNeighbours < 2) {
              changeBit(outputworld, (depth*(IMWD/IntSize)) + offset, x%IntSize, 0);
            }
            else if(liveNeighbours > 3) {
              changeBit(outputworld, (depth*(IMWD/IntSize)) + offset, x%IntSize, 0);
            }
            else if((liveNeighbours == 3) || (liveNeighbours == 2)){
              changeBit(outputworld, (depth*(IMWD/IntSize)) + offset, x%IntSize, 1);
            }
          }
          if(isLiving == 0) {
            if(liveNeighbours == 3) changeBit(outputworld, (depth*(IMWD/IntSize)) + offset, x%IntSize, 1)
            else changeBit(outputworld, (depth*(IMWD/IntSize)) + offset, x%IntSize, 0);
          }
        }
      }

void resetProcessArray(int processArray[3*(IMWD/IntSize)]){
    for (int i = 0; i<3; i++){
        for(int j = 0; j<IMWD/IntSize;j++){
        processArray[i*(IMWD/IntSize)+j] = 0;
      }
    }
}

void processWorker(chanend toDist, chanend pauseWorker, chanend toLEDs, int index){
  int counter = steps;
  int pause = 0;
  int alternator =0;
  while(counter >0 ){
    toDist :> alternator;
    int overWritePart = 2; //stores which part of the array to overwrite with new values
    int processArray[3*(IMWD/IntSize)];
    int top = 0;
    int mid = 1;
    int bottom = 2;
    int depth = 0;
    int outputworld[ubound/noWorkers];
    initialiseOutput(outputworld, processArray);
    for (int y = 0; y<(IMHT/noWorkers);y++){
        recieveOverlap(toDist, processArray);
        transformPixel(index, processArray, depth, mid, top, bottom, outputworld, pauseWorker, pause, toLEDs, alternator);
        depth = depth + 1;
        mid = (mid + 1) % 3;
        top = (top + 1) % 3;
        bottom = (bottom + 1) % 3;
        resetProcessArray(processArray);
    }
    sendOutput(toDist, outputworld);
}
}


//////////////////////////////////////////////////////////
////////////////////DISTRIBUTOR///////////////////////////
//////////////////////////////////////////////////////////

/*void sendOverlap(chanend worker[noWorkers], int world[ubound]){
    for(int index = 0; index < noWorkers; index ++){
              int top;
              top = ((index*(IMHT/noWorkers) -1));
              //bottom = ((index*IMHT/noWorkers) +(IMHT/noWorkers));
              if (top == -1) top = IMHT-1;
              //if (bottom == IMHT) bottom = 0;
              for (int i = 0; i < (IMWD/32);i++) {
                  worker[index] <: world[(top*(IMWD/32))+i];
                  worker[index] <: world[((index*IMHT)/noWorkers) + i];
              }
          //}
      }
}*/

void sendOverlap(chanend worker[noWorkers], int world[ubound]){

  par (int index = 0; index < noWorkers; index ++){

      for(int y = ((index*IMHT)/noWorkers); y < (((index*IMHT)/noWorkers) + IMHT/noWorkers); y++){
          int top, bottom, mid;
          mid = y;
              top = mid -1;
              bottom = mid +1;
              if (top == -1) top = IMHT-1;
              if (bottom == IMHT) bottom = 0;
              for (int i = 0; i < (IMWD/IntSize);i++) {
                  worker[index] <: world[(top*(IMWD/IntSize))+i];
                  worker[index] <: world[(mid*(IMWD/IntSize))+i];
                  worker[index] <: world[(bottom*(IMWD/IntSize))+i];
              }
          //}
      }
}
}

void distributeWork(chanend worker[noWorkers], int world[ubound]){
    for(int index =0; index<noWorkers;index++){
          for(int y = (((index*IMHT)/noWorkers)+1); y < (((index*IMHT)/noWorkers) + IMHT/noWorkers)+1; y++){
              for(int x = 0; x <(IMWD/IntSize); x++){
//                  printf("REACHED HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE\n");
                  worker[index] <: world[((y%IMWD)*(IMWD/IntSize)) + x];
              }
             worker[index] :> int k;
          }
      }
}

void recieveFinal(chanend worker[noWorkers], int world[ubound]){
    for(int index =0; index<noWorkers;index++){
        int base = index*(IMHT/noWorkers);
        int interval = base + (IMHT/noWorkers);
        for(int y = base; y< interval; y++){
            for(int x = 0; x < (IMWD/IntSize); x++){
                worker[index] :> world[(y*(IMWD/IntSize)) + x];
            }
        }
//        printf("%d RECEIVEFINALLLLLLLLLLLLLLLLLLLLLLLLLLLLL\n", index);
    }
}

void getButton(chanend fromButtons, int alternator){
  int buttonPress = 0;
  fromButtons :> buttonPress;
  fromButtons <: 4 | alternator;
}

void sendAlternator(chanend fromButtons, int alternator, chanend worker[noWorkers]){
  fromButtons <: 0 | alternator;
  if (alternator == 1) alternator = 0;
  else alternator = 1;

  for (int a = 0; a < noWorkers; a++) {
      worker[a] <: alternator;
  }
}
//if anything goes wrong its here
void unpack(int world[ubound], chanend c_out){
    for (int y = 0; y< IMHT; y++){
        for(int x =0; x<(IMWD/IntSize); x++){
            for (int k = 0; k < IntSize; k++) {
                uchar temp;
                //int up = (y*(IMWD/32)) + x;
                //printf("%d \n", up);
                //int k = world[up];
                if(isAlive(world[(y*(IMWD/IntSize)) + x],k%IntSize)){
                     temp = 255;
                }
                else temp = 0;
                c_out <: temp;
            }
       }
//        printf("%d UNPAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAACK\n", y);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Start your implementation by changing this function to implement the game of life
// by farming out parts of the image to worker threads who implement it...
// Currently the function just inverts the image
//
/////////////////////////////////////////////////////////////////////////////////////////
void distributor(chanend c_in, chanend c_out, chanend fromAcc, chanend worker[noWorkers], chanend fromButtons)
{
  int counter = steps;
  while (counter > 0) {
  timer keepTime;
  unsigned long long int time = 0;
  unsigned long long int time1 = 0;
  unsigned long long int oneSecond = 100000000;
  int alternator = 1;
  int world[ubound];
//  printf("%d\n",IntSize);
  for (int i = 0; i < ubound; i++) {
      world[i] = 0;
  }
  for(int i = 30; i < 36; i++){
//      printf("$value of clean divide %d is: %d\n",i,cleanDivide(i));
  }
  keepTime :> time;
  printf("THE TIME IN THE DISTRIBUTOR IS %llu \n \n", time);

  int wer = 1 << 10;

//  printf("%d       fdddddddddddddddddddddddddddddddd", wer);

  //Starting up and wait for tilting of the xCore-200 Explorer
  printf("Waiting for Button Click...\n");

// printf( "Waiting for Board Tilt...\n" );
//  fromAcc :> int value;
  getButton(fromButtons, alternator);
  printf( "\n \n \nProcessing...\n \n \n" );
  pack(c_in, world);
//  printf("QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ\n");
  sendAlternator(fromButtons, alternator, worker);
  sendOverlap(worker, world);
  //loop target
//  distributeWork(worker, world);
  //loop target
//  printf("pppppppppppppppppppppppppppppppppppppppppppppppppppppppppp\n");
  fromButtons <: 2 | alternator;
  recieveFinal(worker,world);
  unpack(world,c_out); ///NEEDS IMPLEMENTING
  //sendFinal(c_out, img);
//  printf("lllllllllllllllllllllllllllllllllllllllllllllllllllllllll\n");
  fromButtons <: 0 | alternator;
  counter = counter - 1;

  printf( "\nOne processing round completed...\n" );

  c_in <: 0;
  c_out <: 0;

  printf( "ProcessImage: Start, size = %dx%d\n", IMHT, IMWD );
    keepTime :> time1;
      printf("THE TIME IN THE DISTRIBUTOR IS NOW %llu \n", time1);
      time1 = time1 - time;
      float f = time1 / oneSecond;
      printf("THE TIME DIFFERENCE WAS %f SECONDS\n", f);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Write pixel stream from channel c_in to PGM image file
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataOutStream(char outfname[], chanend c_in)
{
  int res;
  int counter = steps;
//  int alternator = 0;
  while (counter > 0){
  uchar line[ IMWD ];

  //Open PGM file
  printf( "DataOutStream: Start...\n" );
  res = _openoutpgm( outfname, IMWD, IMHT );
  if( res ) {
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
  counter -= 1;
  printf( "DataOutStream: Done...\n" );
  //if (alternator == 0){
  //  strcpy(outfname, "test---.pgm");
  //  alternator = 1;
  //}
  //else {
  //  strcpy(outfname, "testout.pgm");
  //  alternator = 0;
  //}
  c_in :> int data;
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
    if (!tilted) {
      if (x<-20) {
        //tilted = 1 - tilted;
        tilted = 0;
//        toDist <: 1;
      }
    }
    if (!tilted){
      if(x <- 20){
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
      if(x>20){
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

/*char infname[] = "64x64.pgm";     //put your input image path here*/
/*char outfname[] = "64_test_out_does_this_work_vIX.pgm"; //put your output image path here*/
chan c_inIO, c_outIO, c_control;    //extend your channel definitions here
chan worker[noWorkers];
chan buttonCom;
chan LEDButtonCom;
chan pauseWorker[noWorkers];
chan LEDWorkerComm[noWorkers];

par {
    on tile[0] : i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing orientation data
    on tile[1] : orientation(i2c[0],c_control, pauseWorker);        //client thread reading orientation data
    on tile[0] : DataInStream(infname, c_inIO);          //thread to read in a PGM image
    on tile[0] : DataOutStream(outfname, c_outIO);       //thread to write out a PGM image
    on tile[1] : distributor(c_inIO, c_outIO, c_control, worker, buttonCom);//thread to coordinate work on image
    on tile[0] : buttonListener(buttons, buttonCom, LEDButtonCom);
    on tile[0] : showLEDs(leds, LEDButtonCom, LEDWorkerComm);

    par(int index = 0; index<noWorkers; index++){
    on tile[index % 2] : processWorker(worker[index], pauseWorker[index], LEDWorkerComm[index], index);
    }
}


  return 0;
}
