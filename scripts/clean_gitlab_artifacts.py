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


def get_abs_day(year, month, day):
    """
    Calculates absolute day from year 0
    """
    return (int(year) - 1) * 365 + (int(month) - 1) * 31 + int(day)


def get_all_jobs_or_pipelines(input_type : str):
    assert(input_type == "jobs" or input_type == "pipelines")
    print("Collecting {}...".format(input_type))
    project = gl.projects.get(proj_id)
    result_list = []
    
    page = 1
    while True:
        print("Collecting {} from page: {}".format(input_type, page))
        type_attr = getattr(project, input_type)
        page_list = type_attr.list(page=page)
        if len(page_list) <= 0:
            break

        result_list += page_list
        page += 1

    print("Done! Total {} collected: {}".format(input_type, len(result_list)))
    return result_list


def cleanup_artifacts(older_than : int):
    print("Cleaning artifacts...")
    td = date.today()
    cur_day = get_abs_day(td.year, td.month, td.day)

    for job in get_all_jobs_or_pipelines("jobs"):
        print("Cleaning artifacts of job: {}".format(job.id))
        # Avoid erasing unfinished jobs
        if job.finished_at is None:
            continue

        # Absolute number of days when job finished
        [art_year, art_month, art_day] = job.finished_at.split('T')[0].split('-')
        art_days = get_abs_day(art_year, art_month, art_day)

        if (art_days + older_than) < cur_day:
            print("Erasing artifacts of job: {} from: {}".format(job.id, job.finished_at))
            try:
                job.delete_artifacts()
            except:
                print("Could not erase artifacts of job: {}".format(job.id))


def cleanup_pipelines(older_than : int):
    for pipeline in get_all_jobs_or_pipelines("pipelines"):
        print("Cleaning pipeline of job: {}".format(pipeline.id))
        td = date.today()
        cur_day = get_abs_day(td.year, td.month, td.day)

        if pipeline.created_at is None:
            continue

        # Absolute number of days when job finished
        [art_year, art_month, art_day] = pipeline.created_at.split('T')[0].split('-')
        art_days = get_abs_day(art_year, art_month, art_day)

        if (art_days + older_than) < cur_day:
            print("Erasing pipeline: {} from: {}".format(pipeline.id, pipeline.created_at))
            try:
                pipeline.delete()
            except:
                print("Could not erase pipeline: {}".format(pipeline.id))


def cleanup_jobs(older_than : int):
    all_jobs = get_all_jobs_or_pipelines("jobs")
    td = date.today()
    cur_day = get_abs_day(td.year, td.month, td.day)

    for job in all_jobs:
        # Avoid erasing unfinished jobs
        if job.finished_at is None:
            continue

        # Absolute number of days when job finished
        [art_year, art_month, art_day] = job.finished_at.split('T')[0].split('-')
        print("Year: {}, Month: {}, Day: {}".format(art_year, art_month, art_day))
        art_days = get_abs_day(art_year, art_month, art_day)

        if (art_days + older_than) < cur_day:
            print("Erasing job: {}".format(job.id))
            try:
                job.erase()
            except:
                print("Could not erase a job: {}".format(job.id))


if __name__ == "__main__":

    print(sys.argv)
    if len(sys.argv) != 4:
        print("The script shall be called with arguments: <TOKEN> <OLDER_THAN> <CLEAN_MASK>")
        print("     <TOKEN> Gitlab access token")
        print("     <OLDER_THAN> Items older than <OLDER_THAN> days will be cleaned")
        print("     <CLEAN_MASK> Items to be erased (decimal number)")
        print("         Bit 0   - Erase artifacts")
        print("         Bit 1   - Erase pipelines")
        print("         Bit 2   - Erase jobs")

        sys.exit()

    token = sys.argv[1]
    older_than = sys.argv[2]
    mask = sys.argv[3]

    clean_artifacts = False
    clean_pipelines = False
    clean_jobs = False

    if int(mask, 16) & 0x1:
        clean_artifacts = True
        print("Script will erase artifacts older than: {} days".format(older_than))
    if int(mask, 16) & 0x2:
        clean_pipelines = True
        print("Script will erase pipelines older than: {} days".format(older_than))
    if int(mask, 16) & 0x4:
        clean_jobs = True
        print("Script will erase jobs older than: {} days".format(older_than))

    if not older_than.isdigit():
        print("<OLDER_THAN> should be integer!")
        sys.exit()

    connect_to_server()

    if clean_artifacts:
        cleanup_artifacts(int(older_than))
    if clean_pipelines:
        cleanup_pipelines(int(older_than))
    if cleanup_jobs:
        cleanup_jobs(int(older_than))

    sys.exit()

    for job_id in range(int(from_job), int(to_job)):
        cmd = 'curl --request DELETE --header "PRIVATE-TOKEN: {}" "{}/projects/{}/pipelines/{}"'.format(\
               token, repo_url, proj_id, job_id)
        print(cmd)
        os.system(cmd)
        print('\n')

    
