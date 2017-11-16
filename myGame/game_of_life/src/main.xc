// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo code)

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"

#define  IMHT 16                  //image height
#define  IMWD 16                  //image width
#define  noWorkers 2
#define steps 10

typedef unsigned char uchar;      //using uchar as shorthand

on tile[0] : port p_scl = XS1_PORT_1E;         //interface ports to orientation
on tile[0] : port p_sda = XS1_PORT_1F;

char infname[] = "test---.pgm";     //put your input image path here
char outfname[] = "testout.pgm"; //put your output image path here

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
    //fromButtons :> pattern;   //receive new pattern from visualiser
    //p <: pattern;                //send pattern to LED port
    //printf("KKKKKKKKKKKKKKKKKKKKKKKKKKEEEEEEEEEEEEEEEEEEEEEEEEPPPPPPPPPPPPPPPPPPPPPPPSSSSSSSSSS\n");
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
  //sw1 = 14
  //sw2 = 13
  int r;
  int data;
  //int firstTurn = 1;
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
          //printf("BRRRRRRRRRRRAAAAAAAAAAAAAAAAAAAAAAAAPPPPPPPPPPPPPPPPPPPPPPPPP\n");
          toDist :> data;
          //printf(" %d   BRRRRRRRRRRRAAAAAAAAAAAAAAAAAAAAAAAAPPPPPPPPPPPPPPPPPPPPPPPPP\n", data);
          toLEDs <: data;
          //printf("  2   BRRRRRRRRRRRAAAAAAAAAAAAAAAAAAAAAAAAPPPPPPPPPPPPPPPPPPPPPPPPP\n");
          toDist :> data;
          //printf("   3   BRRRRRRRRRRRAAAAAAAAAAAAAAAAAAAAAAAAPPPPPPPPPPPPPPPPPPPPPPPPP\n");
          toLEDs <: data;
          //printf("   4     BRRRRRRRRRRRAAAAAAAAAAAAAAAAAAAAAAAAPPPPPPPPPPPPPPPPPPPPPPPPP\n");
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
  while(counter > 0) {
      int res;
      uchar line[ IMWD ];
      printf( "DataInStream: Start...\n" );

      //Open PGM file
      res = _openinpgm( infname, IMWD, IMHT );
      if( res ) {
          printf( "DataInStream: Error openening %s\n.", infname );
          return;
      }

      for( int y = 0; y < IMHT; y++ ) {
          _readinline( line, IMWD );
          for( int x = 0; x < IMWD; x++ ) {
              c_out <: line[ x ];
      //printf( "-%4.1d ", line[ x ] ); //show image values
          }
      //printf( "\n" );
      }

      //Close PGM image file
      _closeinpgm();
      printf( "DataInStream: Done...\n" );
      counter = counter - 1;
      if (alternator == 0) {
          strcpy(infname,"testout.pgm");
          alternator = 1;
      }
      else {
          strcpy(infname,"test---.pgm");
          alternator = 0;
      }
      c_out :> int data;
  }
  return;
}

int getLiveNeighbours(uchar img[IMWD][IMHT/noWorkers], uchar top[IMWD],uchar bottom[IMWD], int x, int y){
    int counter = 0;
    int flagup =0;
    int flagdown =0;
    for(int j = -1; j<2; j++){
        for(int i =-1; i<2; i++){
            int xneighbour = (x+i) ;
            int yneighbour = (y+j) ;
            if(xneighbour == -1) xneighbour = IMWD -1;
            if(xneighbour == IMWD) xneighbour = 0;
            if(yneighbour == -1) {yneighbour = IMHT-1; flagup =1;}
            if(yneighbour == IMHT/noWorkers) {yneighbour = 0; flagdown = 1;}
            if((i != 0) || (j!=0)){
                if(flagup){
                    if(top[xneighbour] == 255) counter++;
                    flagup =0;
                }
                else if(flagdown){
                    if(bottom[xneighbour] == 255) counter++;
                    flagdown =0;
                }
                else{
                    if (img[xneighbour][yneighbour] == 255) counter++;
                }
            }
        }
    }
    return counter;
}


void processWorker(chanend toDist, int index, chanend pauseWorker, chanend toLEDs){
    int counter  = steps;
    int pause = 0;
    int alternator = 0;
    while (counter > 0){
        uchar img[IMWD][IMHT/noWorkers];
        uchar outimg[IMWD][IMHT/noWorkers];
        uchar top[IMWD]; uchar bottom[IMWD];
        int liveNeighbours;
        toDist :> alternator;
        //printf("lineeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee 1\n");
        for (int x = 0; x <IMWD; x++){
            toDist :> top[x];
            toDist :> bottom[x];
//          printf("$ top[%d], bottom[%d]\n", top[x],bottom[x]);
        }
        //printf("lineeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee 2\n");
        for(int y= 0;y< IMHT/noWorkers;y++){
            for(int x=0;x<IMWD;x++){
                toDist :> img[x][y];
            }
        }
        //printf("lineeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee 3\n");
        for(int y= 0; y<IMHT/noWorkers; y++){
            for(int x=0;x<IMHT;x++){
                outimg[x][y]=img[x][y];
//              printf(" [%d]",outimg[x][y]);
            }
//          printf("\n");
        }
        //printf("lineeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee 4\n");

        for(int y = 0; y<IMHT/noWorkers;y++){
            for(int x =0; x < IMWD; x++){

                pauseWorker :> pause;
                while (pause == 1) {
                    pauseWorker :> pause;
                    toLEDs <: 8 | alternator;
                }
                toLEDs <: 0 | alternator;

                liveNeighbours = getLiveNeighbours(img,top,bottom, x,y);
                if(img[x][y] == 255){
                    if(liveNeighbours < 2) {
                        outimg[x][y] = img[x][y] ^ 0xFF;
                    }
                    else if(liveNeighbours > 3) {
                        outimg[x][y] = img[x][y] ^ 0xFF;
                    }
                }
                if(img[x][y] == 0) {
                    if(liveNeighbours == 3) outimg[x][y] = 255;
                }

            }
        }

        for (int y = 0; y<IMHT/noWorkers;y++){
            for(int x=0;x<IMWD;x++){
                toDist <: outimg[x][y];
            }
        }
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
    int alternator = 1;
    while (counter > 0) {
        uchar img[IMWD][IMHT];
          uchar outimg[IMWD][IMHT];

          //Starting up and wait for tilting of the xCore-200 Explorer
          printf( "ProcessImage: Start, size = %dx%d\n", IMHT, IMWD );
          printf( "Waiting for Button Click...\n" );
         // fromAcc :> value;
          //printf( "%d    Processing...\n", value );
          int buttonPress = 0;
          fromButtons :> buttonPress;
          fromButtons <: 4 | alternator;
          printf( "Processing...\n" );


          for( int y = 0; y < IMHT; y++ ) {   //go through all lines
            for( int x = 0; x < IMWD; x++ ) { //go through each pixel per line
              c_in :> img[x][y];                    //read the pixel value

            }
          }

          fromButtons <: 0 | alternator;
          if (alternator == 1) alternator = 0;
          else alternator = 1;

          for(int y= 0; y< IMHT; y++){
              for(int x=0; x< IMWD; x++){
                  outimg[x][y] = img[x][y];
              }
          }

          for (int a = 0; a < noWorkers; a++) {
              worker[a] <: alternator;
          }

          for(int x = 0; x< IMWD;x++){
              worker[0] <: img[x][15];
              worker[0] <: img[x][8];
              worker[1] <: img[x][7];
              worker[1] <: img[x][0];
          }

          par{
              for(int y =0; y<(IMHT/noWorkers); y++){
                  for (int x = 0; x<IMWD;x++){
                      worker[0] <: img[x][y];
                  }
              }
              for(int y =IMHT/noWorkers; y<IMHT; y++){
                  for (int x = 0; x<IMWD;x++){
                      worker[1] <: img[x][y];
                  }
              }
          }

          fromButtons <: 2 | alternator;

          //printf("they can come to my yard innit\n");

          for(int index =0; index<noWorkers;index++){
              //printf("they can come to my yard innittttttttt\n");
              int base = index*(IMHT/noWorkers);
              int interval = base + (IMHT/noWorkers);
              for(int y = base; y< interval; y++){
                  for(int x = 0; x < IMWD; x++){
                      //printf("%d %d %d UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUu\n", index, base, x);
                      worker[index] :> outimg[x][y];
                      c_out <: outimg[x][y];
                  }
              }
          }

          //printf("theyssssss can come to my yard innit\n");

          fromButtons <: 0 | alternator;

          counter = counter - 1;
          printf( "\nOne processing round completed...\n" );
          c_in <: 0;
          c_out <: 0;
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
  int alternator = 0;
  while (counter > 0) {
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
  counter = counter - 1;
  printf( "DataOutStream: Done...\n" );
  if (alternator == 0) {
            strcpy(outfname, "test---.pgm");
            alternator = 1;
        }
        else {
            strcpy(outfname,"testout.pgm");
            alternator = 0;
        }
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

    //printf("%d is my acceleration \n", x);

    //send signal to distributor after first tilt
    if (!tilted) {
      if (x<-20) {
        tilted = 1 - tilted;
        for (int i = 0; i < noWorkers; i++) {

        }
        tilted = 0;
      }
      //else {
        //toDist <: 1;
      //}
    }
    if (!tilted) {
        if (x < -20) {
            tilted = 1;
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
        if (x > 20) {
            tilted = 0;
            for (int i = 0; i < noWorkers; i++) {
                pauseWorker[i] <: 0;
            }
        }
        else {
            for (int i = 0; i < noWorkers; i++) {
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

chan c_inIO, c_outIO, c_control;    //extend your channel definitions here
chan worker[noWorkers];
chan buttonCom;
chan LEDButtonCom;
chan pauseWorker[noWorkers];
chan LEDWorkerComm[noWorkers];
par {
    on tile[1] : processWorker(worker[0],0, pauseWorker[0], LEDWorkerComm[0]);
    on tile[1] : processWorker(worker[1],1, pauseWorker[1],LEDWorkerComm[1]);
    on tile[0] : i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing orientation data
    on tile[1] : orientation(i2c[0],c_control, pauseWorker);        //client thread reading orientation data
    on tile[0] : DataInStream(infname, c_inIO);          //thread to read in a PGM image
    on tile[0] : DataOutStream(outfname, c_outIO);       //thread to write out a PGM image
    on tile[1] : distributor(c_inIO, c_outIO, c_control, worker, buttonCom);//thread to coordinate work on image
    on tile[0] : buttonListener(buttons, buttonCom, LEDButtonCom);
    on tile[0] : showLEDs(leds, LEDButtonCom, LEDWorkerComm);
  }


  return 0;
}
