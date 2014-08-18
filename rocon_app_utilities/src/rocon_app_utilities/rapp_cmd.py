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

from .dependencies import DependencyChecker
from .rapp_repositories import build_index, get_combined_index, get_index, get_index_dest_prefix_for_base_paths, is_index, load_uris, sanitize_uri, save_uris, uri2url

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
    #  Parse command arguments
    args = argv[2:]
    parser = argparse.ArgumentParser(description='Displays rapp information')
    parser.add_argument('-u', '--uri', nargs='?', help='Optional narrow down list from specific Rapp repository')
    parser.add_argument('-c', '--compatibility', default='rocon://', help='Optional compatibility check')

    parsed_args = parser.parse_args(args)

    if not parsed_args.uri:
        index = get_combined_index()
    else:
        uri = sanitize_uri(parsed_args.uri)
        index = get_index(uri)

    compatible_rapps, incompatible_rapps, invalid_rapps = index.get_compatible_rapps(uri=parsed_args.compatibility, ancestor_share_check=False)

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

    index = get_combined_index()

    rapp = index.get_raw_rapp(resource_name)

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

    index = get_combined_index()
    try:
        rapp = index.get_rapp(resource_name)
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

    index = get_combined_index()
    compatible_rapps, incompatible_rapps, invalid_rapps = index.get_compatible_rapps(compatibility)

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


def _rapp_cmd_install(argv):
    #  Parse command arguments
    args = argv[2:]
    parser = argparse.ArgumentParser(description='Install a list of rapps')
    parser.add_argument('--debug', action='store_true', help='Output debug information')
    parser.add_argument('rapp_names', type=str, nargs='+', help='Rocon URI')

    parsed_args = parser.parse_args(args)
    rapp_names = set(parsed_args.rapp_names)

    index = get_combined_index()

    dependencyChecker = DependencyChecker(index)

    dependencies = dependencyChecker.check_rapp_dependencies(rapp_names)

    missing_dependencies = []
    for rapp_name, deps in dependencies.items():
        missing_dependencies.extend(deps.noninstallable)
    missing_dependencies = set(missing_dependencies)

    noninstallable_rapps = [rapp_name for rapp_name, deps in dependencies.items() if deps.noninstallable]
    if noninstallable_rapps:
        print('Error - The following rapps cannot be installed: %s. Missing dependencies: %s' % (' '.join(noninstallable_rapps.keys()),
                                                                                                 ' '.join(missing_dependencies)
                                                                                                ))
    else:
        # resolve deps and install them
        print("Installing dependencies for: %s" % (' '.join(sorted(rapp_names))))
        if parsed_args.debug:
            print("- installing the following packages: %s" % (' '.join(sorted(set([d for deps in dependencies.values() for d in deps.installable])))))
            print("- already installed packages: %s" % (' '.join(sorted(set([d for deps in dependencies.values() for d in deps.installed])))))
        dependencyChecker.install_rapp_dependencies(rapp_names)


def _rapp_cmd_index(argv):
    #  Parse command arguments
    args = argv[2:]
    parser = argparse.ArgumentParser(description='Generate an index for a Rapp tree')
    parser.add_argument('packages_path', type=str, help='Path to a Rapp tree')
    parser.add_argument('-o', '--outfile', help='Output file name')

    parsed_args = parser.parse_args(args)
    packages_path = parsed_args.packages_path
    outfile_name = parsed_args.outfile

    index_path(packages_path, outfile_name)


def index_path(packages_path, outfile_name):
    index = build_index([packages_path])
    base_path = os.path.dirname(packages_path)
    filename_prefix = outfile_name if outfile_name else os.path.basename(packages_path)
    dest_prefix = os.path.join(base_path, filename_prefix)
    index.write_tarball(dest_prefix)


def _rapp_cmd_add_repository(argv):
    #  Parse command arguments
    args = argv[2:]
    parser = argparse.ArgumentParser(description='Add a rapp repository')
    parser.add_argument('repository_url', type=str, help='URL of a Rapp repository index or a local folder')

    parsed_args = parser.parse_args(args)
    repository_url = parsed_args.repository_url

    uris = load_uris()
    if repository_url in uris:
        raise RuntimeError("'%s' is already listed as a rapp repository" % repository_url)
    repository_url = sanitize_uri(repository_url)
    if os.path.isdir(repository_url) or os.path.isfile(repository_url):
        repository_url = os.path.abspath(repository_url)
    uris.append(repository_url)
    save_uris(uris)
    update_indices()


def _rapp_cmd_remove_repository(argv):
    #  Parse command arguments
    args = argv[2:]
    parser = argparse.ArgumentParser(description='Remove a rapp repository')
    parser.add_argument('repository_url', type=str, help='URL of a Rapp repository index or a local folder')

    parsed_args = parser.parse_args(args)
    repository_url = parsed_args.repository_url

    uris = load_uris()
    if repository_url not in uris:
        raise RuntimeError("'%s' is not listed as a rapp repository" % repository_url)
    uris.remove(repository_url)
    save_uris(uris)
    update_indices()


def _rapp_cmd_list_repositories(argv):
    #  Parse command arguments
    args = argv[2:]
    parser = argparse.ArgumentParser(description='List rapp repositories')

    parser.parse_args(args)

    uris = load_uris()
    for uri in uris:
        print(uri)


def _rapp_cmd_update_repository_indices(argv):
    #  Parse command arguments
    args = argv[2:]
    parser = argparse.ArgumentParser(description='Update indices of rapp repositories')

    parser.parse_args(args)

    update_indices()


def update_indices():
    uris = load_uris()
    for uri in uris:
        # existing indices must not be updated
        if is_index(uri):
            continue
        url = uri2url(uri)
        index = build_index(url)
        dest_prefix = get_index_dest_prefix_for_base_paths(url)
        index.write_tarball(dest_prefix)


def _fullusage():
    print("""\nrocon_app is a command-line tool for printing information about Rapp

Commands:
\trocon_app list\t\tdisplay a list of cached rapps
\trocon_app info\t\tdisplay rapp information
\trocon_app rawinfo\tdisplay rapp raw information
\trocon_app compat\tdisplay a list of rapps that are compatible with the given rocon uri
\trocon_app install\tinstall a list of rapps
\trocon_app add-repo\tadd a rapp repository
\trocon_app remove-repo\tremove a rapp repository
\trocon_app list-repos\tlist the rapp repositories
\trocon_app update\tupdate the indices for the rapp repositories
\trocon_app index\t\tgenerate an index file of a Rapp tree
\trocon_app help\t\tUsage

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
        elif command == 'install':
            _rapp_cmd_install(argv)
        elif command == 'index':
            _rapp_cmd_index(argv)
        elif command == 'add-repo':
            _rapp_cmd_add_repository(argv)
        elif command == 'remove-repo':
            _rapp_cmd_remove_repository(argv)
        elif command == 'list-repos':
            _rapp_cmd_list_repositories(argv)
        elif command == 'update':
            _rapp_cmd_update_repository_indices(argv)
        elif command == 'help':
            _fullusage()
        else:
            _fullusage()
    except RuntimeError as e:
        sys.stderr.write('%s\n' % e)
        sys.exit(1)
    except Exception as e:
        sys.stderr.write("Error: %s\n" % str(e))
        ex, val, tb = sys.exc_info()
        traceback.print_exception(ex, val, tb)

        sys.exit(1)
