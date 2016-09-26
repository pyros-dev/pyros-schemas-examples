#
# This ros message definition gathers all supported HTTP headers for both request and response
# All header fields are optional (only Host is mandatory in HTTP/1.1)
# Follow https://en.wikipedia.org/wiki/Robustness_principle
# TODO : splitting ?
# Ref : https://en.wikipedia.org/wiki/List_of_HTTP_header_fields

# User-Agent is automatically filled by python-request. maybe it doesnt make sense to expose...
string[] User_Agent                   # python-requests/2.10.0 "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Ubuntu Chromium/51.0.2704.79 Chrome/51.0.2704.79 Safari/537.36"     "Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:48.0) Gecko/20100101 Firefox/48.0"

string[] Accept                       # "*/*"                  "text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,*/*;q=0.8"                                                                "text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8"
string[] Accept_Encoding              # "gzip, deflate"        "gzip, deflate, sdch"                                                                                                                       "gzip, deflate"
string[] Accept_Language              #                        "en-US,en;q=0.8"                                                                                                                            "en-US,en;q=0.5"

# this is for advance feature we do not support yet
#string[] Cache_Control                #                                                                                                                                                                    "max-age=0"
#string[] Cookie                       #                                                                                                                                                                    "_ga=GA1.2.13884453.1450081587; _gat=1"

# Host is automatically filled by python-request. maybe it doesnt make sense to expose...
string[] Host                         # "httpbin.org"          "httpbin.org"                                                                                                                               "httpbin.org"

string[] Referer                      #                                                                                                                                                                    "http://httpbin.org/"
string[] Upgrade_Insecure_Requests    #                        "1"                                                                                                                                         "1"

# Access_control_allow_credentials
# access_control_allow_origin
# connection
# content_length
# content_type
# date
# server


# TODO : This should be in same package as HttpStatusCode