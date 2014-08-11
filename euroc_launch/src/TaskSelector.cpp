/*
 * This is a wrapper for EuRoC task selector. It recursively kills it's child processes when terminated with Ctrl-C
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <string>
#include <sstream>

__pid_t pid;

void killRecursive(int killMe)
{
  char buf[16];
  FILE *pgrep;
  std::string cmd("pgrep -P ");
  std::stringstream out;
  out << killMe;
  cmd += out.str();
  if ((pgrep = popen(cmd.c_str(), "r")) == NULL)
    return;
  while (!feof(pgrep))
  {
    if (fgets(buf, 16, pgrep) != NULL)
    {
      killRecursive(atoi(buf));
    }

  }
  kill(killMe, SIGKILL);
}

void killEuroc(int signal)
{
  killRecursive(pid);
  exit(signal);
}

int main(int argc, char **argv)
{
  signal(SIGINT, killEuroc);
  pid = fork();
  if (pid == 0)
  {
    execvp("/opt/euroc_c2s1/start_euroc_task_selector", argv);
  }
  else
  {
    while (1)
    {
      sleep(1);
    }
  }
}
