#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
#################################################################################

from __future__ import division, print_function

import sys
import os
import traceback
import argparse

from .indexer import RappIndexer

#################################################################################
# Global variables
#################################################################################

NAME = 'rocon_app'

#################################################################################
# Local methods
#################################################################################


def _rapp_cmd_list(argv):
    """
      Command-line parsing for 'rapp list' command.
    """
    indexer = RappIndexer()
    compatible_rapps, incompatible_rapps, invalid_rapps = indexer.get_compatible_rapps(ancestor_share_check=False)

    print('== Available Rapp List == ')
    for n in compatible_rapps.values():
        print('  Resource: %s'%(str(n.resource_name)))
        print('     - Compatibility : %s '%str(n.data['compatibility']))
        print('     - Ancestor      : %s '%str(n.ancestor_name))


    if len(invalid_rapps) > 0:
        print('== Invalid Rapp List == ')
        for k, v in invalid_rapps.items():
            print('  ' + k + ' : ' + str(v))


def _rapp_cmd_raw_info(argv):
    print("Displays rapp raw information")
    #  Parse command arguments
    args = argv[2:]
    parser = argparse.ArgumentParser(description='Displays rapp information')
    parser.add_argument('resource_name', type=str, help='Rapp name')

    parsed_args = parser.parse_args(args)
    resource_name = parsed_args.resource_name

    indexer = RappIndexer()

    rapp = indexer.get_raw_rapp(resource_name)

    print('== %s =='%str(rapp))
    for k, v in rapp.raw_data.items():
        print('  %s : %s'%(str(k),str(v)))

def _rapp_cmd_info(argv):
    print("Displays rapp resolved information")
    #  Parse command arguments
    args = argv[2:]
    parser = argparse.ArgumentParser(description='Displays rapp information')
    parser.add_argument('resource_name', type=str, help='Rapp name')

    parsed_args = parser.parse_args(args)
    resource_name = parsed_args.resource_name

    indexer = RappIndexer()
    try:
        rapp = indexer.get_rapp(resource_name)
        print('== %s =='%str(rapp))
        for k, v in rapp.raw_data.items():
            print('  %s : %s'%(str(k),str(v)))
    except Exception as e:
        print('%s : Error - %s'%(resource_name,str(e)))

        
#def _rapp_cmd_depends(argv):
#    print("Dependecies")
#    pass


def _rapp_cmd_depends_on(argv):
    print("Childs")
    pass


#def _rapp_cmd_profile(argv):
#    indexer = RappIndexer()
#    pass


def _rapp_cmd_compat(argv):
    #  Parse command arguments
    args = argv[2:]
    parser = argparse.ArgumentParser(description='Displays list of compatible rapps')
    parser.add_argument('compatibility', type=str, help='Rocon URI')

    parsed_args = parser.parse_args(args)
    compatibility = parsed_args.compatibility

    indexer = RappIndexer()
    compatible_rapps, incompatible_rapps, invalid_rapps = indexer.get_compatible_rapps(compatibility)

    print('== Available Rapp List for [%s] == ' % compatibility)
    for r in compatible_rapps.values():
        print('  Resource: %s'%(str(r.resource_name)))
        print('     - Ancestor : %s '%str(r.ancestor_name))

    print('== Incompatible Rapp List for [%s] == ' % compatibility)
    for k, v in incompatible_rapps.items(): 
        print('  ' + k + ' : ' + str(v.raw_data['compatibility']))

    print('== Invalid Rapp List for [%s] == ' % compatibility)
    for k, v in invalid_rapps.items(): 
        print('  ' + k + ' : ' + str(v))

def _fullusage():
    print("""\nrocon_app is a command-line tool for printing information about Rapp

Commands:
\trocon_app list\tdisplay a list of cached rapps
\trocon_app info\tdisplay rapp information
\trocon_app rawinfo\tdisplay rapp raw information
\trocon_app compat\tdisplay a list of rapps that are compatible with the given rocon uri
\trocon_app help\tUsage

Type rocon_app <command> -h for more detailed usage, e.g. 'rocon_app info -h'
""")
    sys.exit(getattr(os, 'EX_USAGE', 1))


# Future TODO    
#\trocon_app depends\tdisplay a rapp dependency list
#\trocon_app depends-on\tdisplay a list of rapps that depend on the given rapp
#\trocon_app profile\tupdate cache


#################################################################################
# Main
#################################################################################

def main():
    argv = sys.argv

    # process argv
    if len(argv) == 1:
        _fullusage()
    try:
        command = argv[1]
        if command == 'list':
            _rapp_cmd_list(argv)
        elif command == 'info':
            _rapp_cmd_info(argv)
        elif command == 'rawinfo':
            _rapp_cmd_raw_info(argv)
        elif command == 'depends':
            _rapp_cmd_depends(argv)
        elif command == 'depends-on':
            _rapp_cmd_depends_on(argv)
        elif command == 'profile':
            _rapp_cmd_profile(argv)
        elif command == 'compat':
            _rapp_cmd_compat(argv)
        elif command == 'help':
            _fullusage()
        else:
            _fullusage()
    except Exception as e:
        sys.stderr.write("Error: %s\n" % str(e))
        ex, val, tb = sys.exc_info()
        traceback.print_exception(ex, val, tb)

        sys.exit(1)
