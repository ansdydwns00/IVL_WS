function helperCallbackImage(msg)
   
global g_img

g_img = [];

g_img = rosReadImage(msg);
    
end