# content of conftest.py

import subprocess
import os
from time import sleep
import signal
from py.xml import html
import pytest

executable = '/home/allen/Downloads/brown-sugar/build/ubuntu/brown-sugar'

pro = None


def pytest_html_results_table_header(cells):
    cells.insert(1, html.th("Description"))
    cells.pop()


def pytest_html_results_table_row(report, cells):
    cells.insert(1, html.td(report.description))
    cells.pop()


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    outcome = yield
    report = outcome.get_result()
    report.description = str(item.function.__doc__)

def pytest_collection_modifyitems(config, items):
    def param_part(item):
        # check for the wanted module and test class
        if item.nodeid.startswith("hmi_test"):
            # find the start of the parameter part in the nodeid
            index = item.nodeid.find('[')
            if index > 0:
                # sort by parameter name
                return item.name[item.nodeid.index('['):]

        # for all other cases, sort by node id as usual
        return item.nodeid
    
def pytest_configure(config):
    """
    Allows plugins and conftest files to perform initial configuration.
    This hook is called for every plugin and initial conftest
    file after command line options have been parsed.
    """


def pytest_sessionstart(session):
    """
    Called after the Session object has been created and
    before performing collection and entering the run test loop.
    """
    global pro
    # pro = subprocess.Popen([executable], stdout=subprocess.PIPE, shell=False, preexec_fn=os.setsid)
    sleep(2)


def pytest_sessionfinish(session, exitstatus):
    """
    Called after whole test run finished, right before
    returning the exit status to the system.
    """
    global pro
    # os.killpg(os.getpgid(pro.pid), signal.SIGTERM)


def pytest_unconfigure(config):
    """
    called before test process is exited.
    """