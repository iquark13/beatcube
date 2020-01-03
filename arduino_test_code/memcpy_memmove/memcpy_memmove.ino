int a1[10] = {1,1,1,1,1,1,1,1,1,1};
int a2[10] = {0};

long int timer = 0;

const int howbig = 1000;
float big[howbig]={0};
float bigHolder[howbig]={0};



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Serial.println("A1:");
  for (int i = 0; i<10; ++i){
    Serial.println(a1[i]);
  }
  Serial.println("A2:");
  for (int i = 0; i<10; ++i){
    Serial.println(a2[i]);
  }

  Serial.println("memcpy standard:");

  memcpy(a2,a1,sizeof(a1));
  for (int i = 0; i<10; ++i){
    Serial.println(a2[i]);
    a2[i]=0; //reset a2
  }

  Serial.println("Resetting to be half 5, half 0");
  for (int i = 0; i<10; ++i){
    a2[i]=i+1;
    a1[i]=11+i;
    Serial.println(a2[i]);
  }
  Serial.println("A1 setup:");
  for (int i =0;i<10;++i){
    Serial.println(a1[i]);
  }

  Serial.println("memmove shift 3 places down:");

  // 3 = number to shift
  // 10 = size of array
  memmove(a2,a2+(3),(10-3)*sizeof(*a2));
  memcpy(a2+(10-3),a1,3*sizeof(*a2));
  
  for (int i =0;i<10;++i){
    Serial.println(a2[i]);
  }

  for (int i =0; i<howbig;++i){
    big[i]=i;
    bigHolder[i]=1;
  }

  timer=micros();
  memcpy(big,bigHolder,howbig*sizeof(*bigHolder));
  timer=micros()-timer;

  Serial.print("How long did it take: ");
  Serial.println(timer);
  
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
