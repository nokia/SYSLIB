//*****************************************************************************
//
// Copyright 2017 Nokia, All Rights Reserved
//
//*****************************************************************************

#include <arpa/inet.h>
#include <assert.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <ti/apps/netfp_proxy/netfp_proxy.h>

#define GROUP_NAME_LEN         20
#define PATH_MAX_LEN           256
#define L3QOS_CHANNEL_TBL_SIZE 60

static const char* QOS_TREE_BASE_NAME = "/proc/device-tree/qos-tree-5";

typedef struct L3QosChannel
{
    char         name[GROUP_NAME_LEN];
    unsigned int input_queue;
} L3QosChannel;

static L3QosChannel L3_QOS_CHANNEL_TABLE[L3QOS_CHANNEL_TBL_SIZE];

static int shouldOmitDir(const char* dirName)
{
    size_t dirNameLen = strlen(dirName);
    return ( strncmp(dirName, ".", dirNameLen) == 0 || strncmp(dirName, "..", dirNameLen) == 0 );
}

static unsigned int readQueueNumber(const char* inputQueueFilePath)
{
    int fd = open(inputQueueFilePath, O_RDONLY);
    if(fd < 0)
    {
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error opening file \"%s\"[%d]: [%s]", inputQueueFilePath, errno, strerror(errno));
        assert(0);
    }

    unsigned int value = 0;
    if(sizeof(value) != read(fd, &value, sizeof(value)))
    {
        close(fd);
        NetfpProxy_logMsg (NETFP_PROXY_LOG_ERROR, "Error reading from \"%s\"[%d]: [%s]", inputQueueFilePath, errno, strerror(errno));
        assert(0);
    }

    close(fd);

    return ntohl(value);
}

static void parseMinorGroup(const char* groupMajorName, const char* groupMinorName)
{
    static int channelTableCounter = 0;

    char groupMinorPath[PATH_MAX_LEN] = {0};
    assert(snprintf(groupMinorPath, PATH_MAX_LEN, "%s/%s/%s", QOS_TREE_BASE_NAME, groupMajorName, groupMinorName) < PATH_MAX_LEN);

    DIR *groupMinorDirectory = NULL;
    if((groupMinorDirectory = opendir(groupMinorPath)) == NULL)
        return;

    struct dirent *groupMinorDirEntry = NULL;
    while((groupMinorDirEntry = readdir(groupMinorDirectory)) != NULL)
    {
        if(shouldOmitDir(groupMinorDirEntry->d_name))
            continue;

        if(strncmp(groupMinorDirEntry->d_name, "input-queues", strlen(groupMinorDirEntry->d_name)) != 0)
            continue;

        char inputQueueFilePath[PATH_MAX_LEN] = {0};
        assert(snprintf(inputQueueFilePath, PATH_MAX_LEN, "%s/%s", groupMinorPath, groupMinorDirEntry->d_name) < PATH_MAX_LEN);

        assert(channelTableCounter < L3QOS_CHANNEL_TBL_SIZE);
        L3QosChannel* channelTableEntry = &L3_QOS_CHANNEL_TABLE[channelTableCounter];

        assert(snprintf(channelTableEntry->name, sizeof(channelTableEntry->name),
                        "%s/%s", groupMajorName, groupMinorName) < GROUP_NAME_LEN);
        channelTableEntry->input_queue = readQueueNumber(inputQueueFilePath);

        channelTableCounter++;
    }

    closedir(groupMinorDirectory);
}

static void parseMajorGroup(const char* groupMajorName)
{
    char groupMajorPath[PATH_MAX_LEN] = {0};
    assert(snprintf(groupMajorPath, PATH_MAX_LEN, "%s/%s", QOS_TREE_BASE_NAME, groupMajorName) < PATH_MAX_LEN);

    DIR *groupMajorDirectory = NULL;
    if((groupMajorDirectory = opendir(groupMajorPath)) == NULL)
        return;

    struct dirent *groupMajorDirEntry = NULL;
    while((groupMajorDirEntry = readdir(groupMajorDirectory)) != NULL)
    {
        if(shouldOmitDir(groupMajorDirEntry->d_name))
            continue;

        parseMinorGroup(groupMajorName, groupMajorDirEntry->d_name);
    }

    closedir(groupMajorDirectory);
}

int populateL3QosChannelTable(void)
{
    DIR *baseDirectory = NULL;

    if((baseDirectory = opendir(QOS_TREE_BASE_NAME)) == NULL)
        return -1;

    struct dirent *baseDirEntry = NULL;
    while((baseDirEntry = readdir(baseDirectory)) != NULL)
    {
        if(shouldOmitDir(baseDirEntry->d_name))
            continue;

        parseMajorGroup(baseDirEntry->d_name);
    }

    closedir(baseDirectory);

    return 0;
}

int lookupL3QosQueue(const char* groupName, uint16_t* qnum)
{
    int groupNameLen = strlen(groupName);
    for(int i = 0; i < L3QOS_CHANNEL_TBL_SIZE; i++)
    {
        if(strncmp(groupName, L3_QOS_CHANNEL_TABLE[i].name, groupNameLen) == 0)
        {
            *qnum = (uint16_t)L3_QOS_CHANNEL_TABLE[i].input_queue;
            return 0;
        }
    }

    return -1;
}
