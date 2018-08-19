import os

head = open('urdf-template/atlas_head.txt', 'r').read()
body = open('urdf-template/atlas_body.txt', 'r').read()
tail = open('urdf-template/atlas_tail.txt', 'r').read()

max_num_row = 5     # ADJUST
out_dir = 'output/atlas'    # ADJUST

if not os.path.exists(out_dir):
    os.makedirs(out_dir)

for numrow in range(1, max_num_row+1):
    text = head + '\n'

    print('generate URDF for numrow={}...'.format(numrow))

    # body
    i = 0
    for row in range(0, numrow):
        for col in range(0, numrow):
            base_pos = (row * 2.0, col * 2.0, 1.0)

            text += body.format(
                base_pos_0 = base_pos[0],
                base_pos_1 = base_pos[1],
                base_pos_2 = base_pos[2],
                id = i
            )

            i+=1

    # tail
    text += '\n'
    text += tail

    # save
    print('save URDF for numrow={}...'.format(numrow))

    with open(os.path.join(out_dir,
                           "robot{}.urdf".format(numrow)), "w") as text_file:
        text_file.write(text)

print('Atlas URDF generate finished.')