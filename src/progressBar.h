#ifndef _PROGRESSBAR_H_
#define _PROGRESSBAR_H_

#include <iostream>

class TProgressBar{
public:
  TProgressBar() : barWidth_(60) {}
  void setBarWidth(int bw) {this->barWidth_=bw;};
  void update() {
    float progress = (counter++)/(file_size);
    printf("Progress: [");
    int pos = barWidth_ * progress;
    for (int i = 0; i < barWidth_; ++i) {
        if (i < pos) printf("=");
        else if (i == pos) printf(">");
        else printf(" ");
    }
    printf("] %.2f %  \r", float(progress * 100.0));
    fflush(stdout);
  }
  void end() {update(); printf("\n"); counter = 0;};
  void setSize(double s){file_size = s;};


private:
  int barWidth_{70};
  double counter{};
  double file_size{};
};

#endif
