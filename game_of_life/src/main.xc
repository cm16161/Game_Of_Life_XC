// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo code)

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"

#define  ImageSize 16
#define  IMHT ImageSize                  //image height
#define  IMWD ImageSize                  //image width
#define  noWorkers 4

#define  steps 100

#if(ImageSize % 32 == 0)
    #define IntSize 32
#else
    #define IntSize 16
#endif

#define ubound ((IMHT * IMWD) / IntSize)

typedef unsigned char uchar;      //using uchar as shorthand

on tile[0] : port p_scl = XS1_PORT_1E;         //interface ports to orientation
on tile[0] : port p_sda = XS1_PORT_1F;

char infname[20] = "test.pgm";// = fuckUpXc();//strcat(strcat(strcat("64","x"), "64"), ".pgm");     //put your input image path here
char outfname[20] = "junk.pgm"; //put your output image path here

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

int showLEDs(out port p, chanend fromDist, chanend workerPause[noWorkers]) {
  int pattern = 0; //1st bit...separate green LED
               //2nd bit...blue LED
               //3rd bit...green LED
               //4th bit...red LED
  while (1) {
    select {
        case workerPause[int i] :> pattern:
            p <: pattern;
            break;
        case fromDist :> pattern:
            p <: pattern;
            break;
    }
  }
  return 0;
}

void buttonListener(in port b, chanend toDist) {
  int r;
  int started = 0;
  while (started == 0) {
           b when pinseq(14)  :> r;    // check that no button is pressed
           toDist <: 0; //send out that a button was pushed
           started = 1;
  }
  while (1) {
      select {
          case b when pinseq(13) :> r:
              toDist <: 1;
              break;
          default:
              toDist <: 0;
              break;
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
    int res;
    uchar line[ IMWD ];
    printf( "DataInStream: Start...\n" );
    res = _openinpgm( infname, IMWD, IMHT );
    if( res ) {
      printf( "DataInStream: Error openening %s\n.", infname );
      return;
    }
  for( int y = 0; y < IMHT; y++ ) {
    _readinline( line, IMWD );
    for( int x = 0; x < IMWD; x++ ) {
      c_out <: line[ x ];
    }
  }
  _closeinpgm();
  printf( "DataInStream: Done...\n" );
  counter = counter - 1;
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

int liveCellCount(int outputworld[ubound/noWorkers]){
    int liveCell = 0;
    for (int y = 0; y< IMHT/noWorkers; y++){
        for(int x =0; x<(IMWD/IntSize); x++){
            for (int k = 0; k < IntSize; k++) {
                if(isAlive(outputworld[(y*(IMWD/IntSize)) + x],k%IntSize)){
                     liveCell++;
                }
            }
       }
    }
return liveCell;
}

int getLiveNeighbours(int index, int processArray[3*(IMWD/IntSize)], int top, int bottom , int mid, int x){
int counter = 0;
int flag;
for(int j = -1; j<2; j++){
    for(int i =-1; i<2; i++){
        flag = 0;
        int xneighbour = (x+i) ;
        int yneighbour;
        if(xneighbour == -1) xneighbour = IMWD -1;
        if(xneighbour == IMWD) xneighbour = 0;
        if(j == -1) {yneighbour = top; flag =1;}
        else if(j== 1) {yneighbour = bottom; flag = -1;}
        else if(j == 0) {yneighbour = mid;}
        if((i != 0) || (j!=0)){
            int passTop = cleanDivide(xneighbour);
            if(flag == 1){
                if(isAlive(processArray[passTop], xneighbour % IntSize)) counter++;
                flag =0;
            }
            else if(flag == -1){
                int passBottom = cleanDivide(xneighbour);
                if(isAlive(processArray[(2*(IMWD/IntSize))+passBottom], xneighbour % IntSize)) counter++;
                flag =0;
            }
            else{
                int passY = cleanDivide(xneighbour);
                if(isAlive(processArray[((IMWD/IntSize))+passY], xneighbour % IntSize)) counter++;
            }
        }
    }
}
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

void recieveWork(chanend toDist, int processArray[3*(IMWD/IntSize)]){
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

/*void pauseWorkerFunction(chanend pauseWorker, int pause, chanend toLEDs, int alternator){
  pauseWorker :> pause;
//  printf("$PAUSEWORKER FUNCTION!!!!!!!!!!!!!!!!!!!!!!!\n");
  while (pause == 1) {
        printf("$PAUSEWORKER FUNCTION!!!!!!!!!!!!!!!!!!!!!!!\n");
      pauseWorker :> pause;
      toLEDs <: 8 | alternator;
  }
  toLEDs <: 0 | alternator;
}*/

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
            if(liveNeighbours == 3) changeBit(outputworld, (depth*(IMWD/IntSize)) + offset, x%IntSize, 1);
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

void processWorker(chanend toDist,chanend pauseWorker, chanend toLEDs, int index, chanend toTimer){
  int counter = steps;
  int pause = 0;
  int alternator =0;
  int rounds =0;
  float time = 0;
  while(counter >0){
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
        pauseWorker :> pause;
        if(pause ==1){
            printf("Number of Rounds Processed: %d\n", (rounds));
            printf("Number of Live Cells Processed by Worker %d: %d\n",index, liveCellCount(outputworld));
            if(index == 0) {
                printf("Time elapsed is: %f\n", time/(float)1000000000);
            }
        }
        while (pause == 1) {
            pauseWorker :> pause;
            toLEDs <: 8 | alternator;
        }
        pause = 0;
        toLEDs <: 0 | alternator;
        recieveWork(toDist, processArray);
        transformPixel(index, processArray, depth, mid, top, bottom, outputworld, pauseWorker, pause, toLEDs, alternator);
        depth = depth + 1;
        mid = (mid + 1) % 3;
        top = (top + 1) % 3;
        bottom = (bottom + 1) % 3;
        resetProcessArray(processArray);
    }
    rounds++;
    sendOutput(toDist, outputworld);
    toTimer :> time;
}
}

void distributeWork(chanend worker[noWorkers], int world[ubound]){
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
    }
}

void getButton(chanend fromButtons, chanend display, int alternator){
  int buttonPress = 0;
  fromButtons :> buttonPress;
  display <: 4 | alternator;
}

void sendAlternator(int alternator, chanend worker[noWorkers]){
  if (alternator == 1) alternator = 0;
  else alternator = 1;

  for (int a = 0; a < noWorkers; a++) {
      worker[a] <: alternator;
  }
}
void unpack(int world[ubound], chanend c_out){
    for (int y = 0; y< IMHT; y++){
        for(int x =0; x<(IMWD/IntSize); x++){
            for (int k = 0; k < IntSize; k++) {
                uchar temp;
                if(isAlive(world[(y*(IMWD/IntSize)) + x],k%IntSize)){
                     temp = 255;
                }
                else temp = 0;
                c_out <: temp;
            }
       }
    }
}

void initialiseWorld(int world[ubound]){
    for (int i = 0; i < ubound; i++) {
          world[i] = 0;
      }
}

void distributor(chanend toLEDs, chanend c_in, chanend c_out, chanend fromAcc, chanend worker[noWorkers], chanend fromButtons, chanend toTimer)
{
  int counter = steps;
  float time = 0;
  float oneSecond = 100000000;
  int alternator = 0;
  int world[ubound];
  int receival = 1;
  printf("Waiting for Button Click...\n");
  initialiseWorld(world);
  /*for (int i = 0; i < ubound; i++) {
      world[i] = 0;
  }*/
  getButton(fromButtons, toLEDs, alternator);
  pack(c_in, world);
  toLEDs <: 0 | alternator;
  while (counter > 0) {
      printf( "Processing...\n" );
      toTimer <: 1;
      sendAlternator(alternator, worker);
      distributeWork(worker, world);
      recieveFinal(worker,world);
      toTimer <: 0;
      toTimer :> time;
      printf("THE TIME DIFFERENCE WAS %lf SECONDS\n", time/oneSecond);
      fromButtons :> receival;
      if ((receival == 1) || (counter ==1)) {toLEDs <: 2 | alternator;
      unpack(world,c_out);}
      toLEDs <: 0 | alternator;
      counter = counter - 1;
      printf( "\nOne processing round completed...\n" );
  }
}



///////////////////////////////////////////////
///////////////  TIMER  ///////////////////////
///////////////////////////////////////////////

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

/////////////////////////////////////////////////////////////////////////////////////////
//
// Write pixel stream from channel c_in to PGM image file
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataOutStream(char outfname[], chanend c_in)
{
  int res;
//  int counter = steps;
//  while (counter > 1){
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
//    printf( "DataOutStream: Line written...\n" );
  }
  //Close the PGM image
  _closeoutpgm();
//  counter -= 1;
  printf( "DataOutStream: Done...\n" );
//}
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
      if(x < -20){
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
chan pauseWorker[noWorkers];
chan LEDWorkerComm[noWorkers];
chan distTimer;
chan LEDsDist;
chan workerTimer[noWorkers];

par {
    on tile[0] : i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing orientation data
    on tile[1] : orientation(i2c[0],c_control, pauseWorker);        //client thread reading orientation data
    on tile[0] : DataInStream(infname, c_inIO);          //thread to read in a PGM image
    on tile[0] : DataOutStream(outfname, c_outIO);       //thread to write out a PGM image
    on tile[1] : distributor(LEDsDist, c_inIO, c_outIO, c_control, worker, buttonCom, distTimer);//thread to coordinate work on image
    on tile[0] : buttonListener(buttons, buttonCom);
    on tile[0] : showLEDs(leds, LEDsDist, LEDWorkerComm);
    on tile[0] : recordTime(distTimer, workerTimer);

    par(int index = 0; index<noWorkers; index++){
    on tile[index % 2] : processWorker(worker[index], pauseWorker[index], LEDWorkerComm[index], index, workerTimer[index]);
    }
}


  return 0;
}
