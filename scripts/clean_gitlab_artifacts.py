#!/usr/bin/python3

import os
import sys
import subprocess
import gitlab
from datetime import date

repo_url = "https://gitlab.fel.cvut.cz"
proj_id = "6719"
token = ""
gl = None


def connect_to_server():
    global gl
    gl = gitlab.Gitlab(repo_url, token)
    print("Gitlab API version: {}".format(gl.api_version))
    print("Gitlab API URL: {}".format(gl.api_url))
    print("Gitlab version: {}".format(gl.version))


def cleanup_artifacts(older_than : int):
    project = gl.projects.get(proj_id)
    page = 1
    jobs = project.jobs.list(page=page)

    while (len(jobs) > 0):
        print("Cleaning jobs on page: {}".format(page))

        # Absolute number of days - today
        td = date.today()
        cur_day = (td.year - 1) * 365 + (td.month - 1) * 31 + td.day

        for job in jobs:
            if (job.finished_at == None):
                continue
            
            # Absolute number of days when job finished
            [art_year, art_month, art_day] = job.finished_at.split('T')[0].split('-')
            art_days = (int(art_year) - 1) * 365 + (int(art_month) - 1) * 31 + int(art_day)

            # TODO: Check that artifacts actually exist before trying to erase them!
            if ((art_days + older_than) < cur_day):
                print("Erasing artifacts of job: {} from: {}".format(job.id, job.finished_at))
                job.delete_artifacts()

        page += 1
        jobs = project.jobs.list(page=page)


def cleanup_pipelines(older_than : int):
    project = gl.projects.get(proj_id)
    page = 1
    pipelines = project.pipelines.list(page=page)

    while (len(pipelines) > 0):
        print("Cleaning pipelines on page: {}".format(page))
    
        # Absolute number of days - today
        td = date.today()
        cur_day = (td.year - 1) * 365 + (td.month - 1) * 31 + td.day

        for pipeline in pipelines:
            if (pipeline.created_at == None):
                continue
            
            # Absolute number of days when job finished
            [art_year, art_month, art_day] = pipeline.created_at.split('T')[0].split('-')
            art_days = (int(art_year) - 1) * 365 + (int(art_month) - 1) * 31 + int(art_day)

            if ((art_days + older_than) < cur_day):
                print("Erasing pipeline: {} from: {}".format(pipeline.id, pipeline.created_at))
                pipeline.delete()

        page += 1
        pipelines = project.pipelines.list(page=page)


if __name__ == "__main__":

    print(sys.argv)
    if (len(sys.argv) != 4):
        print("The script shall be called with arguments: <TOKEN> <OLDER_THAN> <CLEAN_MASK>")
        print("     <TOKEN> Gitlab access token")
        print("     <OLDER_THAN> Items older than <OLDER_THAN> days will be cleaned")
        print("     <CLEAN_MASK> Items to be erased")
        print("         Bit 0   - Erase artifacts")
        print("         Bit 1   - Erase pipelines")

        sys.exit();

    token = sys.argv[1]
    older_than = sys.argv[2]
    mask = sys.argv[3]

    clean_artifacts = False
    clean_pipelines = False

    if (int(mask, 16) & 0x1):
        clean_artifacts = True
        print("Script will erase artifacts older than: {} days".format(older_than))
    if (int(mask, 16) & 0x2):
        clean_pipelines = True
        print("Script will erase pipelines older than: {} days".format(older_than))

    if ((older_than.isdigit() == False)):
        print("<OLDER_THAN> should be integer!")
        sys.exit();

    connect_to_server()

    if (clean_artifacts):
        cleanup_artifacts(int(older_than))

    if (clean_pipelines):
        cleanup_pipelines(int(older_than))

    sys.exit()

    for job_id in range(int(from_job), int(to_job)):
        cmd = 'curl --request DELETE --header "PRIVATE-TOKEN: {}" "{}/projects/{}/pipelines/{}"'.format(\
               token, repo_url, proj_id, job_id)
        print(cmd)
        os.system(cmd)
        print('\n')

    
