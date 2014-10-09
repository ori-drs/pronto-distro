// file: bot-lcm-splice.c
// desc: utility to merge a set of logs into a single one

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <regex.h>
#include <getopt.h>

#include <glib.h>

#include <lcm/lcm.h>

static void
usage()
{
    printf(
            "usage: bot-lcm-logsplice [OPTIONS] <source_logfile1> <source_logfile2> [source_logfile3...] <dest_logfile>\n"
                "\n"
                "Splice together (filtered) channels from two or more source\n"
                "log files, and generate a single destination log file.\n"
                "\n"
                "Options:\n"
                "  -h        prints this help text and exits\n"
                "  -c CHAN   POSIX regular expression.  Channels matching this expression\n"
                "            will be copied to the destination logfile.  Defaults to .* if\n"
                "            left unspecified.\n"
                "  -i        invert the regular expression CHAN, so that only channels not\n"
                "            matching CHAN will be copied to the destination logfile.\n"
                "  -s START  start time.  Messages logged less than START seconds\n"
                "            after the first message in the logfile will not be\n"
                "            extracted.\n"
                "  -e END    end time.  Messages logged more than END seconds\n"
                "            after the first message in the logfile will not be\n"
                "            extracted.\n"
                "  -v        verbose mode. Prints a summary of channels extracted\n");
    exit(1);
}

static void
_verbose_entry_summary(gpointer key, gpointer value, gpointer user_data)
{
    printf("%20s: %d\n", (char*) key, *((int*) value));
}

int
main(int argc, char **argv)
{
    int verbose = 0;
    int filterChannels =0;
    char *pattern = strdup(".*");
    char *dest_fname = NULL;
    int64_t start_utime = 0;
    int64_t end_utime = -1;
    int have_end_utime = 0;
    int invert_regex = 0;

    char *optstring = "hc:vs:e:i";
    int c;

    while ((c = getopt_long(argc, argv, optstring, NULL, 0)) >= 0) {
        switch (c)
            {
        case 'h':
            usage();
            break;
        case 's':
            {
                char *eptr = NULL;
                double start_time = strtod(optarg, &eptr);
                if (*eptr != 0)
                    usage();
                start_utime = (int64_t) (start_time * 1000000);
            }
            break;
        case 'e':
            {
                char *eptr = NULL;
                double end_time = strtod(optarg, &eptr);
                if (*eptr != 0)
                    usage();
                end_utime = (int64_t) (end_time * 1000000);
                have_end_utime = 1;
            }
            break;
        case 'i':
            invert_regex = 1;
            filterChannels = 1;
            break;
        case 'c':
            free(pattern);
            pattern = strdup(optarg);
            filterChannels = 1;
            break;
        case 'v':
            verbose = 1;
            break;
        default:
            usage();
            break;
            };
    }

    if (start_utime < 0 || (have_end_utime && end_utime < start_utime))
        usage();

    if (optind > argc - 3)
        usage();

    regex_t preg;
    if (0 != regcomp(&preg, pattern, REG_NOSUB | REG_EXTENDED)) {
        fprintf(stderr, "bad regex\n");
        exit(1);
    }

    int num_src_logs = argc - optind - 1;
    fprintf(stderr, "Splicing together %d logs\n", num_src_logs);
    lcm_eventlog_t *src_logs[num_src_logs];
    for (int i = 0; i < argc - optind - 1; i++) {
        char * src_fname = argv[optind + i];
        src_logs[i] = lcm_eventlog_create(src_fname, "r");
        if (!src_logs[i]) {
            perror("Unable to open source logfile");
            for (int j = 0; j < i; j++)
                lcm_eventlog_destroy(src_logs[j]);
            regfree(&preg);
            return 1;
        }
    }

    dest_fname = argv[argc - 1];

    lcm_eventlog_t *dst_log = lcm_eventlog_create(dest_fname, "w");
    if (!dst_log) {
        perror("Unable to open destination logfile");
        for (int i = 0; i < num_src_logs; i++)
            lcm_eventlog_destroy(src_logs[i]);
        regfree(&preg);
        return 1;
    }

    GHashTable *counts = g_hash_table_new_full(g_str_hash, g_str_equal, free,
            free);
    int nwritten = 0;
    int have_first_event_timestamp = 0;
    int64_t first_event_timestamp = -1;

    lcm_eventlog_event_t *events[num_src_logs];
    for (int i = 0; i < num_src_logs; i++)
        events[i] = lcm_eventlog_read_next_event(src_logs[i]);
    while (1) {
        lcm_eventlog_event_t *event;
        int mind = -1;
        int64_t mtime = INT64_MAX;
        for (int i = 0; i < num_src_logs; i++) {
            if (events[i] == NULL)
                continue;
            else if (events[i]->timestamp < mtime) {
                mtime = events[i]->timestamp;
                mind = i;
            }
        }
        if (mind < 0) //all are null
            break;

        event = events[mind];
        events[mind] = lcm_eventlog_read_next_event(src_logs[mind]);

        if (!have_first_event_timestamp) {
            first_event_timestamp = event->timestamp;
            have_first_event_timestamp = 1;
        }

        int64_t elapsed = event->timestamp - first_event_timestamp;
        if (elapsed < start_utime) {
            lcm_eventlog_free_event(event);
            continue;
        }
        if (have_end_utime && elapsed > end_utime) {
            lcm_eventlog_free_event(event);
            break;
        }

        int copy_to_dest = 1;
        if (filterChannels) {
            int regmatch = regexec(&preg, event->channel, 0, NULL, 0);
            copy_to_dest = (regmatch == 0 && !invert_regex) || (regmatch != 0
                    && invert_regex);
        }
        if (copy_to_dest) {
            lcm_eventlog_write_event(dst_log, event);
            nwritten++;

            if (verbose) {
                int *count = g_hash_table_lookup(counts, event->channel);
                if (!count) {
                    count = (int*) malloc(sizeof(int));
                    *count = 1;
                    g_hash_table_insert(counts, strdup(event->channel), count);
                    printf("matched channel %s\n", event->channel);
                } else {
                    *count += 1;
                }
            }
        }
        lcm_eventlog_free_event(event);
    }

    if (verbose) {
        g_hash_table_foreach(counts, _verbose_entry_summary, NULL);
        printf("=====\n");
        printf("Events written: %d\n", nwritten);
    }

    regfree(&preg);
    for (int i = 0; i < num_src_logs; i++)
        lcm_eventlog_destroy(src_logs[i]);
    lcm_eventlog_destroy(dst_log);
    g_hash_table_destroy(counts);
    return 0;
}
